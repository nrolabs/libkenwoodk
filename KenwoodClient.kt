/*
 * libkenwoodk - Kenwood CAT driver for the iSDR driver host
 *
 * Copyright (C) 2026 Isak Ruas <isakruas@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
package com.isaklab.libkenwoodk

import com.isaklab.isdrdrivers.core.RadioClient
import com.isaklab.isdrdrivers.core.TransmitCapable
import com.isaklab.libcivk.CivTransport
import com.isaklab.libkenwoodk.KenwoodProtocol as P
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong
import kotlin.concurrent.thread
import kotlin.math.abs
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

/**
 * Kenwood rig client: request/answer over serial or the KNS LAN port,
 * AI-driven unsolicited tracking, and bandscope delivery on the spectrum
 * plane.
 *
 * One reader thread owns the transport. Control calls enqueue their command
 * and wait for the answer bearing the matching echo prefix; the reader also
 * folds in everything `AI2` makes the rig volunteer (frequency, mode,
 * keying, meters, scope lines). Set commands have no acknowledge of their
 * own, so a set is confirmed by reading the value back. There is no IQ
 * plane: every data delivery is `onDataReceived(spectrumDb, empty)`.
 */
class KenwoodClient(
    transport: CivTransport,
    private val link: Link,
    /** KNS (account, password); required on LAN, unused on serial. */
    private val credentials: Pair<String, String>?,
    /** (power spectrum in dB, interleaved IQ — always empty for this dialect) */
    private val onDataReceived: (FloatArray, FloatArray) -> Unit,
    private val onConnectionStatusChanged: (Boolean, String) -> Unit,
) : RadioClient, TransmitCapable {

    /**
     * Which physical link the transport carries; the KNS handshake, the `##`
     * command family and the keepalive exist only on LAN.
     */
    enum class Link { SERIAL, LAN }

    companion object {
        /** How long one request waits for its answer before retrying, in ms. */
        private const val REPLY_TIMEOUT_MS = 300L

        /** Attempts per request. */
        private const val ATTEMPTS = 3

        /**
         * The KNS server drops a connection after 10 s of silence; half of
         * that is the keepalive cadence.
         */
        const val KEEPALIVE_MS = 5_000L

        private val EMPTY_FLOATS = FloatArray(0)
    }

    private class Pending(val prefix: String) {
        /** Failure = the rig's `?;` rejection. */
        val reply = ArrayBlockingQueue<Result<String>>(1)
    }

    /**
     * Bandscope placement state, polled at connect and refreshed by AI
     * reports; the scope line itself carries no edges.
     */
    private class ScopeState {
        var mode: P.BandscopeMode? = null
        var spanCode: Int? = null
        var limits: Pair<Long, Long>? = null
        var expand = false
    }

    private var transport: CivTransport? = transport

    private val freqHz = AtomicLong(0)

    /** OM mode code, -1 until known. */
    private val modeCode = AtomicInteger(-1)
    private val smeter = AtomicInteger(-1)
    private val spanHz = AtomicLong(0)
    private val lowEdgeHz = AtomicLong(0)
    private val highEdgeHz = AtomicLong(0)
    private val ptt = AtomicBoolean(false)
    private val txAuthorized = AtomicBoolean(false)
    private val running = AtomicBoolean(false)
    private val keepaliveMs = AtomicLong(KEEPALIVE_MS)

    private val scopeLock = Object()
    private val scope = ScopeState()

    private val outgoing = LinkedBlockingQueue<String>()
    private val pendingLock = Object()
    private var pending: Pending? = null

    private var reader: Thread? = null
    private var started = false
    private var model: KenwoodModels.ScopeModel? = null

    @Volatile private var stateListener: (() -> Unit)? = null

    /**
     * Fired when the RIG changes its own frequency (the operator turns the
     * dial and the AI report arrives) — never for tunes this client itself
     * commanded, which the caller already reports.
     */
    @Volatile var onFrequencyChanged: ((Long) -> Unit)? = null

    /** (low edge Hz, high edge Hz) of the sweep that was just delivered. */
    @Volatile var onSpanDelivered: ((Long, Long) -> Unit)? = null

    @Volatile private var spectrumWanted = true

    override var spectrumEnabled: Boolean
        get() = spectrumWanted
        set(value) {
            spectrumWanted = value
            if (model != null) {
                // Also stop the rig streaming lines nobody will draw.
                if (value) {
                    enableScope()
                } else {
                    P.setDd0(0)?.let { outgoing.offer(it) }
                }
            }
        }

    override fun setStateListener(listener: (() -> Unit)?) {
        stateListener = listener
    }

    /**
     * Override the LAN keepalive cadence (default 5 s). Call before
     * connect; exists so tests can exercise the keepalive without waiting
     * wall-clock seconds.
     */
    fun setKeepaliveIntervalMs(ms: Long) {
        keepaliveMs.set(ms.coerceAtLeast(1))
    }

    /** The rig's current OM mode code, -1 until known. */
    fun mode(): Int = modeCode.get()

    /** Last S-meter reading in dots, -1 until known. */
    fun smeter(): Int = smeter.get()

    fun scopeCapable(): Boolean = model != null

    fun modelName(): String = model?.name ?: "Kenwood"

    /** The edges of the last delivered sweep, in Hz. */
    fun scopeEdges(): Pair<Long, Long> = Pair(lowEdgeHz.get(), highEdgeHz.get())

    /** This LAN connection holds the transmit authorization (`##TI1`). */
    fun txAuthorized(): Boolean = txAuthorized.get()

    // ---- request/answer ------------------------------------------------------

    /**
     * Send [command] and wait for the answer starting with [prefix],
     * retrying through timeouts. Null on rejection (`?;`), timeout or
     * teardown.
     */
    private fun transact(command: String, prefix: String): String? {
        if (!running.get()) return null
        repeat(ATTEMPTS) {
            val p = Pending(prefix)
            synchronized(pendingLock) { pending = p }
            outgoing.offer(command)
            val result = p.reply.poll(REPLY_TIMEOUT_MS, TimeUnit.MILLISECONDS)
            synchronized(pendingLock) { if (pending === p) pending = null }
            if (result != null) return result.getOrNull()
        }
        return null
    }

    /** Fire-and-forget: set commands have no acknowledge to wait for. */
    private fun send(command: String) {
        outgoing.offer(command)
    }

    // ---- rig control ---------------------------------------------------------

    /**
     * Set the operating mode from the app's CAT mode-code space.
     *
     * `CMD_CAT_SET_MODE` carries the CI-V code numbering the host already
     * speaks (0 LSB, 1 USB, 2 AM, 3 CW, 4 RTTY/FSK, 5 FM, 7 CW-R,
     * 8 RTTY-R); the dialect keeps one mode-code space app-side and each
     * dialect translates at its own boundary — here to the OM hex digit.
     */
    fun setMode(catMode: Int): Boolean {
        val om = when (catMode) {
            0 -> P.MODE_LSB
            1 -> P.MODE_USB
            2 -> P.MODE_AM
            3 -> P.MODE_CW
            4 -> P.MODE_FSK
            5 -> P.MODE_FM
            7 -> P.MODE_CW_R
            8 -> P.MODE_FSK_R
            else -> return false
        }
        val cmd = P.setOm(0, om) ?: return false
        send(cmd)
        // No acknowledge exists; the read-back is the confirmation.
        val seg = transact(P.getOm(0)!!, "OM") ?: return false
        val (_, m) = P.parseOm(seg) ?: return false
        modeCode.set(m)
        return m == om
    }

    private fun fail(msg: String): Boolean {
        disconnect()
        onConnectionStatusChanged(false, msg)
        return false
    }

    /**
     * KNS session establishment: `##CN` grant, then `##ID` login. The model
     * is not known before login, so the TS-890S encoding is tried first and
     * the TS-990S encoding on rejection.
     */
    private fun knsHandshake(): String? {
        val granted = transact(P.knsConnect(), "##CN")?.let { P.parseKnsConnectReply(it) }
        when (granted) {
            true -> {}
            false -> return "radio refused the connection (busy or KNS disabled)"
            null -> return "no KNS answer on this port"
        }
        val (account, password) = credentials
            ?: return "KNS login needs credentials: open the radio as user:password@host"
        val attempts = listOfNotNull(
            P.knsLoginTs890(true, account, password),
            P.knsLoginTs990(account, password),
        )
        for (login in attempts) {
            val seg = transact(login, "##ID") ?: continue
            if (P.parseKnsLoginReply(seg) == true) return null
        }
        return "KNS login rejected (check user:password@host)"
    }

    /**
     * Poll the bandscope placement so the edges of the edge-less `##DD2` /
     * `DD2` lines can be derived.
     */
    private fun pollScopeState() {
        val bs3 = transact(P.getBs3(), "BS3")?.let { P.parseBs3(it) }
        val bs4 = transact(P.getBs4(), "BS4")?.let { P.parseBs4(it) }
        val bsm = transact(P.getBsm0(), "BSM0")?.let { P.parseBsm0(it) }
        val bso = transact(P.getBso(), "BSO")?.let { P.parseBso(it) }
        synchronized(scopeLock) {
            scope.mode = bs3
            scope.spanCode = bs4
            scope.limits = bsm
            scope.expand = bso ?: false
        }
        bs4?.let { code -> P.bs4SpanHz(code)?.let { spanHz.set(it) } }
        refreshEdges()
    }

    private fun readInitialState() {
        transact(P.getFa(), "FA")?.let { seg ->
            P.parseFa(seg)?.let { freqHz.set(it) }
        }
        transact(P.getOm(0)!!, "OM")?.let { seg ->
            P.parseOm(seg)?.let { (_, m) -> modeCode.set(m) }
        }
    }

    /**
     * The DD0 streaming code this session uses: high-rate LAN cycle on KNS,
     * the unlinked self-describing serial mode otherwise, clamped to what
     * the model accepts.
     */
    private fun dd0Code(): Int? {
        val m = model ?: return null
        val wanted = if (link == Link.LAN) 1 else 5
        return when {
            wanted <= m.dd0Max -> wanted
            // No serial streaming mode on this model: control-only.
            link == Link.SERIAL -> null
            else -> minOf(m.dd0Max, wanted)
        }
    }

    private fun enableScope() {
        dd0Code()?.let { code -> P.setDd0(code)?.let { send(it) } }
    }

    // ---- reader --------------------------------------------------------------

    private fun readerLoop(transport: CivTransport) {
        val splitter = P.Splitter()
        val assembler = P.SerialScopeAssembler()
        val buf = ByteArray(4096)
        val keepalive = keepaliveMs.get()
        var lastWrite = System.nanoTime()
        try {
            while (running.get()) {
                while (true) {
                    val cmd = outgoing.poll() ?: break
                    transport.writeAll(cmd.toByteArray(Charsets.US_ASCII))
                    lastWrite = System.nanoTime()
                }
                if (link == Link.LAN &&
                    (System.nanoTime() - lastWrite) / 1_000_000 >= keepalive
                ) {
                    // The KNS server drops the session after 10 s of silence.
                    transport.writeAll(P.getPs().toByteArray(Charsets.US_ASCII))
                    lastWrite = System.nanoTime()
                }
                val n = transport.readSome(buf)
                if (n == 0) continue
                for (seg in splitter.push(buf, n)) {
                    handleSegment(assembler, seg)
                }
            }
        } catch (_: Exception) {
            if (running.getAndSet(false)) {
                onConnectionStatusChanged(false, "link lost")
            }
        } finally {
            transport.close()
        }
    }

    /** Hand the segment to the waiting request when its echo prefix matches. */
    private fun tryDeliver(seg: String): Boolean {
        synchronized(pendingLock) {
            val p = pending ?: return false
            if (!seg.startsWith(p.prefix)) return false
            pending = null
            p.reply.offer(Result.success(seg))
            return true
        }
    }

    private fun handleSegment(assembler: P.SerialScopeAssembler, segment: P.Segment) {
        val seg = when (segment) {
            is P.Segment.Error -> {
                // `?;` rejects whatever was last sent.
                synchronized(pendingLock) {
                    val p = pending
                    if (p != null) {
                        pending = null
                        p.reply.offer(
                            Result.failure(IllegalStateException("rig rejected the command")),
                        )
                    }
                }
                return
            }
            is P.Segment.Cmd -> segment.body
        }
        val solicited = tryDeliver(seg)

        // Answers and AI reports share one shape; fold every recognized
        // segment into the caches, solicited or not.
        val lanBins = P.parseLanDd2(seg)
        if (lanBins != null) {
            deliverLine(lanBins)
            return
        }
        val serialSplit = P.parseSerialDd2(seg)
        if (serialSplit != null) {
            assembler.push(serialSplit.first, serialSplit.second)?.let { deliverLine(it) }
            return
        }
        val dd4 = P.parseDd4Header(seg)
        if (dd4 != null) {
            val expand = synchronized(scopeLock) { scope.expand }
            dd4.edges(expand)?.let { (low, high) ->
                lowEdgeHz.set(low)
                highEdgeHz.set(high)
                spanHz.set(high - low)
            }
            return
        }
        val fa = P.parseFa(seg)
        if (fa != null) {
            val previous = freqHz.getAndSet(fa)
            if (!solicited && previous != fa) {
                // A rig-side tune: let the host re-announce the truth.
                onFrequencyChanged?.invoke(fa)
                stateListener?.invoke()
            }
            refreshEdges()
            return
        }
        val om = P.parseOm(seg)
        if (om != null) {
            modeCode.set(om.second)
            return
        }
        val tx = P.parseTxState(seg)
        if (tx != null) {
            ptt.set(tx)
            return
        }
        val sm = P.parseSm(seg)
        if (sm != null) {
            smeter.set(sm)
            return
        }
        val ti = P.parseKnsTi(seg)
        if (ti != null) {
            txAuthorized.set(ti)
            return
        }
        val bs3 = P.parseBs3(seg)
        if (bs3 != null) {
            synchronized(scopeLock) { scope.mode = bs3 }
            refreshEdges()
            return
        }
        val bs4 = P.parseBs4(seg)
        if (bs4 != null) {
            synchronized(scopeLock) { scope.spanCode = bs4 }
            val span = P.bs4SpanHz(bs4)
            if (span != null && spanHz.getAndSet(span) != span && !solicited) {
                // A rig-side span change is a changed effective rate.
                stateListener?.invoke()
            }
            refreshEdges()
            return
        }
        val bsm = P.parseBsm0(seg)
        if (bsm != null) {
            synchronized(scopeLock) { scope.limits = bsm }
            refreshEdges()
            return
        }
        val bso = P.parseBso(seg)
        if (bso != null) {
            synchronized(scopeLock) { scope.expand = bso }
            refreshEdges()
        }
    }

    /** Re-derive the scope edges from the cached placement state. */
    private fun refreshEdges() {
        val mode: P.BandscopeMode?
        val span: Long?
        val limits: Pair<Long, Long>?
        val expand: Boolean
        synchronized(scopeLock) {
            mode = scope.mode
            span = scope.spanCode?.let { P.bs4SpanHz(it) }
            limits = scope.limits
            expand = scope.expand
        }
        if (mode == null) return
        val center = freqHz.get()
        P.deriveEdges(mode, center, span, limits, expand)?.let { (low, high) ->
            lowEdgeHz.set(low)
            highEdgeHz.set(high)
        }
    }

    private fun deliverLine(bins: ByteArray) {
        if (bins.size != P.SCOPE_BINS || !spectrumWanted) return
        val low = lowEdgeHz.get()
        val high = highEdgeHz.get()
        if (high <= low) {
            // No axis yet: a spectrum without edges would be drawn on a
            // wrong frequency scale.
            return
        }
        onSpanDelivered?.invoke(low, high)
        onDataReceived(P.binsToDb(bins), EMPTY_FLOATS)
    }

    // ---- RadioClient ---------------------------------------------------------

    override suspend fun connect(): Boolean = withContext(Dispatchers.IO) { connectBlocking() }

    /** Blocking body of [connect]; also directly testable off a coroutine. */
    fun connectBlocking(): Boolean {
        val t: CivTransport
        synchronized(this) {
            if (started) return running.get()
            started = true
            t = transport ?: return false
            transport = null
        }
        running.set(true)
        reader = thread(name = "kenwood-reader") { readerLoop(t) }

        if (link == Link.LAN) {
            knsHandshake()?.let { return fail(it) }
        }

        val id = transact(P.getId(), "ID")?.let { P.parseId(it) }
            ?: return fail("rig did not answer")
        val model = KenwoodModels.scopeModel(id)
        if (model == null) {
            // Classic tier: MD/IF-style control is a different dialect;
            // refuse honestly instead of half-driving the rig.
            return fail(
                "${KenwoodModels.classicName(id)} speaks the classic Kenwood dialect " +
                    "(MD/IF), which is not implemented; supported models are " +
                    "TS-890S and TS-990S",
            )
        }
        this.model = model

        // AI on (volatile, per-connector) — the scope stream and every
        // unsolicited cache update depend on it.
        send(P.setAi2())
        readInitialState()
        pollScopeState()
        enableScope()
        onConnectionStatusChanged(true, model.name)
        return true
    }

    override fun disconnect() {
        if (running.getAndSet(false)) {
            reader?.join()
            reader = null
        }
        // Never started: the transport is still ours to release.
        synchronized(this) {
            transport?.close()
            transport = null
        }
    }

    override fun setFrequency(hz: Long) {
        val cmd = P.setFa(hz) ?: return
        send(cmd)
        // No acknowledge exists for sets; the read-back both confirms and
        // records what the rig actually accepted.
        transact(P.getFa(), "FA")?.let { seg ->
            P.parseFa(seg)?.let { actual ->
                freqHz.set(actual)
                refreshEdges()
            }
        }
    }

    override fun frequencyHz(): Long = freqHz.get()

    override fun setSampleRate(hz: Int) {
        // The "rate" of a scope-only stream is its span. Snap the request
        // onto the BS4 ladder, then trust only the rig's read-back.
        if (model == null) return
        val target = hz.toLong()
        var code = 0
        for (i in P.BS4_SPANS_HZ.indices) {
            if (abs(P.BS4_SPANS_HZ[i] - target) < abs(P.BS4_SPANS_HZ[code] - target)) {
                code = i
            }
        }
        P.setBs4(code)?.let { send(it) }
        transact(P.getBs4(), "BS4")?.let { seg ->
            P.parseBs4(seg)?.let { actual ->
                synchronized(scopeLock) { scope.spanCode = actual }
                P.bs4SpanHz(actual)?.let { spanHz.set(it) }
                refreshEdges()
            }
        }
    }

    override fun sampleRateHz(): Int = spanHz.get().toInt()

    // ---- TransmitCapable -----------------------------------------------------

    override fun setTxFrequency(hz: Long) {
        // The rig transmits where it is tuned; there is no separate TX NCO.
        setFrequency(hz)
    }

    override fun setPtt(on: Boolean) {
        // Keying has no acknowledge and no dedicated query; the optimistic
        // cache is corrected by the AI TX/RX report the change triggers.
        send(if (on) P.setTx(0)!! else P.setRx())
        ptt.set(on)
    }

    override fun submitTxIq(iq: FloatArray) {
        // TX audio rides the rig's own soundcard/VoIP plane; CAT carries
        // only the keying.
    }

    override fun isTransmitting(): Boolean = ptt.get()
}
