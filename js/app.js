"use strict";
const $ = id => document.getElementById(id);
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const fmtHz = f => f >= 10000 ? Math.round(f / 1000) + " kHz" : (f >= 1000 ? Math.round(f / 100) / 10 + " kHz" : Math.round(f) + " Hz");

/* ---------------- constants & params ---------------- */
const NFFT = 1024, TARGET_FS = 16000, WINDB = 80;
const HZ_MIN = 30, HZ_MAX = 8000;
const DEFAULTS = { nlf: -60, minlen: 50, maxlen: 1000, fmin: 200, fmax: 6000, topn: 5 };
const P = { ...DEFAULTS };
const BPQ_DEFAULT = 10;
let bpq = BPQ_DEFAULT;
const MINI_H = 36;          // minimap height in px (bottom of canvas)
const MIN_ZOOM_DUR = 0.01;  // minimum visible window in seconds (10 ms)

/* ---------------- state ---------------- */
let audioCtx = null, fileBuf = null, fname = "audio";
let stft = null, rawImg = null, gatedImg = null;
let segments = [], analysisMs = 0, playSrc = null;
let segT0 = 0;
let gainDb = 0; // input gain applied to the loaded recording (emulates mic sensitivity)

/* mic stream capture state */
let micCapturing = false;
let micAbortCtl = null;
let micReader = null;
let micChunks = [];
let micBytes = 0;
let micPeakLin = 0;
let micMeterTimer = 0;

/* zoom / pan state */
let viewStart = 0, viewEnd = 0;       // visible time window (seconds)
let isPanning = false, panStartX = 0, panStartVS = 0, panStartVE = 0, panMoved = false;
let miniDrag = false, miniDragX = 0, miniDragVS = 0, miniDragVE = 0;
let fullCanvasGated = null, fullCanvasRaw = null;

/* ---------------- FFT ---------------- */
let rev = null;
(function () {
    rev = new Uint32Array(NFFT);
    const LOG = Math.log2(NFFT);
    for (let i = 0; i < NFFT; i++) {
        let r = 0;
        for (let j = 0; j < LOG; j++) r = (r << 1) | (i >> j) & 1;
        rev[i] = r >>> 0;
    }
})();
function fftInPlace(re, im) {
    const n = re.length;
    for (let i = 0; i < n; i++) {
        const j = rev[i];
        if (j > i) {
            let t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (let size = 2; size <= n; size <<= 1) {
        const half = size >> 1;
        const ang = -2 * Math.PI / size;
        const wr0 = Math.cos(ang), wi0 = Math.sin(ang);
        for (let i = 0; i < n; i += size) {
            let wr = 1, wi = 0;
            for (let j = 0; j < half; j++) {
                const a = i + j, b = a + half;
                const vr = re[b] * wr - im[b] * wi;
                const vi = re[b] * wi + im[b] * wr;
                re[b] = re[a] - vr; im[b] = im[a] - vi;
                re[a] = re[a] + vr; im[a] = im[a] + vi;
                const nwr = wr * wr0 - wi * wi0;
                wi = wr * wi0 + wi * wr0; wr = nwr;
            }
        }
    }
}

/* ---------------- resampling ---------------- */
function toMono(buf) {
    const n = buf.length, chs = Math.min(buf.numberOfChannels, 2), out = new Float32Array(n);
    for (let c = 0; c < chs; c++) { const ch = buf.getChannelData(c); for (let i = 0; i < n; i++)out[i] += ch[i] / chs; }
    return out;
}
function lowpass(mono, srcFS, dstFS) {
    const n = mono.length, fc = Math.min(0.45 * dstFS, 0.4 * srcFS) / srcFS;
    const half = 65, len = half * 2 + 1, filt = new Float32Array(len);
    let sum = 0;
    for (let i = 0; i < len; i++) {
        const m = i - half;
        let v = m === 0 ? 2 * fc : Math.sin(2 * Math.PI * fc * m) / (Math.PI * m);
        const w = 0.54 - 0.46 * Math.cos(2 * Math.PI * i / (len - 1));
        filt[i] = v * w; sum += filt[i];
    }
    for (let i = 0; i < len; i++) filt[i] /= sum;
    const out = new Float32Array(n);
    for (let i = 0; i < n; i++) {
        let s = 0; const start = Math.max(0, i - half), end = Math.min(n - 1, i + half);
        for (let k = start; k <= end; k++) s += mono[k] * filt[k - i + half];
        out[i] = s;
    }
    return out;
}
function resampleToTarget(mono, srcFS, dstFS) {
    if (srcFS === dstFS) return mono;
    const lp = lowpass(mono, srcFS, dstFS);
    const n = lp.length, ratio = dstFS / srcFS;
    const len = Math.max(1, Math.round(n * ratio));
    const out = new Float32Array(len), step = srcFS / dstFS;
    for (let i = 0; i < len; i++) {
        const pos = i * step, i0 = pos | 0, f = pos - i0, i1 = Math.min(n - 1, i0 + 1);
        out[i] = lp[i0] * (1 - f) + lp[i1] * f;
    }
    return out;
}

/* ---------------- input gain ---------------- */
function gainLin() { return Math.pow(10, gainDb / 20); }
function scaleGain(mono) {
    const g = gainLin();
    if (g === 1) return mono;
    const out = new Float32Array(mono.length);
    for (let i = 0; i < mono.length; i++) out[i] = mono[i] * g;
    return out;
}

/* ---------------- STFT ---------------- */
function buildSTFT(mono16, fs) {
    const n = mono16.length, hop = NFFT / 2, half = NFFT / 2;
    const ncol = Math.max(1, Math.ceil((n - NFFT) / hop) + 1);
    const win = new Float32Array(NFFT);
    for (let i = 0; i < NFFT; i++) win[i] = 0.5 * (1 - Math.cos(2 * Math.PI * i / NFFT));
    const re = new Float32Array(NFFT), im = new Float32Array(NFFT);
    const frames = new Array(ncol), scale = 2 / NFFT;
    for (let f = 0; f < ncol; f++) {
        re.fill(0);
        im.fill(0);
        const off = f * hop;
        for (let i = 0; i < NFFT; i++) re[i] = win[i] * (off + i < n ? mono16[off + i] : 0);
        fftInPlace(re, im);
        const mag = new Float32Array(half);
        for (let k = 0; k < half; k++) mag[k] = Math.hypot(re[k], im[k]) * scale;
        frames[f] = mag;
    }
    return { frames, ncol, fs, T: hop / fs, db: fs / NFFT, dur: n / fs, mono: mono16 };
}

/* ---------------- spectrogram images ---------------- */
function dbColorLUT() {
    const stops = [[16, 17, 31], [19, 52, 94], [16, 116, 140], [13, 206, 196], [247, 223, 84], [255, 255, 255]];
    const lut = new Uint8ClampedArray(256 * 3);
    for (let i = 0; i < 256; i++) {
        const t = (i / 255) * (stops.length - 1), s = Math.min(stops.length - 2, Math.floor(t)), f = t - s;
        for (let c = 0; c < 3; c++) lut[i * 3 + c] = stops[s][c] + (stops[s + 1][c] - stops[s][c]) * f;
    }
    return lut;
}
const LUT = dbColorLUT();
function makeSpectroImage(w, h, gated) {
    const img = new ImageData(w, h), data = img.data, S = stft, bins = NFFT / 2;
    const top = Math.log(HZ_MIN), lspan = Math.log(HZ_MAX) - top;
    const nlf = gated ? P.nlf : 0;
    const b0 = gated ? clamp(Math.round(P.fmin / S.db), 0, bins - 1) : 0;
    const b1 = gated ? clamp(Math.round(P.fmax / S.db), 0, bins - 1) : bins - 1;
    const hz = new Float32Array(h), kRow = new Int32Array(h), outRow = new Uint8Array(h);
    for (let y = 0; y < h; y++) {
        const hz0 = Math.exp(top + (y + 0.5) / h * lspan);
        const k = clamp(Math.round(hz0 / S.db), 0, bins - 1);
        hz[y] = hz0; kRow[y] = k;
        outRow[y] = gated && (k < b0 || k > b1) ? 1 : 0;
    }
    const lo = gated ? nlf - 6 : -80;
    const span = (0 - lo) || 1;
    for (let y = 0; y < h; y++) {
        const k = kRow[y], out = outRow[y];
        const rowOff = y * w * 4;
        for (let x = 0; x < w; x++) {
            const f = Math.min(S.ncol - 1, (x + 0.5) / w * S.ncol);
            const i0 = f | 0, i1 = Math.min(S.ncol - 1, i0 + 1), fr = f - i0;
            const m = S.frames[i0][k] * (1 - fr) + S.frames[i1][k] * fr;
            const dbm = 10 * Math.log10(m * m + 1e-30);
            let c;
            if (gated && out) c = 0;
            else c = clamp((dbm - lo) / span * 255, 0, 255) | 0;
            const p = rowOff + x * 4;
            data[p] = LUT[c * 3]; data[p + 1] = LUT[c * 3 + 1]; data[p + 2] = LUT[c * 3 + 2]; data[p + 3] = 255;
        }
    }
    return img;
}

/* cache full-size spectrogram on offscreen canvases so we can crop cheaply */
function cacheFullImages() {
    if (!stft) return;
    const W = $("cv").width, H = $("cv").height, plotH = H - MINI_H;
    if (!fullCanvasRaw || fullCanvasRaw.width !== W || fullCanvasRaw.height !== plotH) {
        fullCanvasRaw = document.createElement("canvas");
        fullCanvasRaw.width = W; fullCanvasRaw.height = plotH;
        fullCanvasGated = document.createElement("canvas");
        fullCanvasGated.width = W; fullCanvasGated.height = plotH;
    }
    fullCanvasRaw.getContext("2d").putImageData(rawImg, 0, 0);
    fullCanvasGated.getContext("2d").putImageData(gatedImg, 0, 0);
}

/* ---------------- vertical zoom helpers ---------------- */
function getVisibleLogRange() {
    const logMin = Math.log(HZ_MIN), logMax = Math.log(HZ_MAX);
    const fLo = clamp(P.fmin, HZ_MIN, HZ_MAX), fHi = clamp(P.fmax, HZ_MIN, HZ_MAX);
    let lo = Math.log(Math.min(fLo, fHi)), hi = Math.log(Math.max(fLo, fHi));
    if (hi <= lo) { hi = lo + Math.log(2); } // degenerate guard
    const bandSpan = hi - lo;
    const visibleSpan = bandSpan / 0.8; // band fills ~80% of vertical space
    const margin = (visibleSpan - bandSpan) / 2;
    let vLo = lo - margin, vHi = hi + margin;
    // clamp to absolute limits (keep margin where possible)
    if (vLo < logMin) vLo = logMin;
    if (vHi > logMax) vHi = logMax;
    // if clamped one side, try to keep span where possible, but never exceed full range
    if (vHi - vLo < visibleSpan) {
        // not enough room at edges — just show full range
        if (vLo === logMin && vHi === logMax) { /* full */ }
        else if (vLo === logMin) { vHi = Math.min(logMax, vLo + visibleSpan); }
        else if (vHi === logMax) { vLo = Math.max(logMin, vHi - visibleSpan); }
    }
    return { vLogMin: vLo, vLogMax: vHi, vSpan: vHi - vLo };
}

/* ---------------- view helpers ---------------- */
function resetView() {
    if (!stft) return;
    viewStart = 0; viewEnd = stft.dur;
}
function clampView() {
    if (!stft) return;
    let dur = viewEnd - viewStart;
    if (dur < MIN_ZOOM_DUR) dur = MIN_ZOOM_DUR;
    if (dur > stft.dur) dur = stft.dur;
    if (viewStart < 0) viewStart = 0;
    if (viewStart + dur > stft.dur) viewStart = stft.dur - dur;
    viewEnd = viewStart + dur;
}
function canvasTime(clientX) {
    const cv = $("cv"), rect = cv.getBoundingClientRect();
    const x = (clientX - rect.left) * (cv.width / rect.width);
    return viewStart + x / cv.width * (viewEnd - viewStart);
}
function timeToX(t) { return (t - viewStart) / (viewEnd - viewStart) * $("cv").width; }

/* ---------------- detection ---------------- */
function findPeaks(c0, c1, b0, b1) {
    const bins = NFFT / 2, mean = new Float32Array(bins);
    for (let c = c0; c <= c1; c++) { const fr = stft.frames[c]; for (let k = b0; k <= b1; k++)mean[k] += fr[k]; }
    const cspan = c1 - c0 + 1, cand = [];
    for (let k = b0 + 1; k < b1; k++) {
        const a = mean[k];
        if (a <= mean[k - 1] || a < mean[k + 1]) continue;
        const vL = mean[k - 1] * mean[k - 1], vC = a * a, vR = mean[k + 1] * mean[k + 1];
        const d = vL + vR - 2 * vC; let rel = 0;
        if (Math.abs(d) > 1e-12) rel = clamp(0.5 * (vL - vR) / d, -0.9, 0.9);
        cand.push({ k: k + rel, db: 10 * Math.log10(vC / (cspan * cspan) + 1e-30) });
    }
    cand.sort((a, b) => b.db - a.db);
    const chosen = [];
    for (const cc of cand) { if (chosen.every(s => Math.abs(s.k - cc.k) >= 2)) chosen.push(cc); if (chosen.length >= P.topn) break; }
    const peaks = chosen.map(cc => ({
        f: Math.round((cc.k + 0.5) * stft.db / 5) * 5,
        db: Math.round(clamp(cc.db, -WINDB, 0) * 10) / 10
    }));
    let maxLin = 0;
    for (let c = c0; c <= c1; c++) { const fr = stft.frames[c]; for (let k = b0; k <= b1; k++) { const m = fr[k]; if (m > maxLin) maxLin = m; } }
    return { peaks, peakDb: 10 * Math.log10(maxLin * maxLin + 1e-30) };
}
function analyze() {
    const t0 = performance.now(), S = stft, bins = NFFT / 2;
    const b0 = clamp(Math.round(P.fmin / S.db), 0, bins - 1);
    const b1 = clamp(Math.round(P.fmax / S.db), 0, bins - 1);
    const active = new Uint8Array(S.ncol);
    for (let f = 0; f < S.ncol; f++) {
        const fr = S.frames[f];
        for (let k = b0; k <= b1; k++) { if (10 * Math.log10(fr[k] * fr[k] + 1e-30) >= P.nlf) { active[f] = 1; break; } }
    }
    const minDur = P.minlen / 1000;
    const MIN_DIP_FRAMES = 1;
    const edge = new Uint8Array(active);
    {
        let f = 0;
        while (f < active.length) {
            if (!active[f]) {
                let g = f; while (g < active.length && !active[g]) g++;
                if (g - f < MIN_DIP_FRAMES) { for (let x = f; x < g; x++) edge[x] = 1; }
                f = g;
            } else f++;
        }
    }
    const segs = [];
    let s = -1;
    for (let f = 0; f <= S.ncol; f++) {
        const on = f < S.ncol && edge[f];
        if (on && s < 0) s = f;
        else if (!on && s >= 0) { if ((f - s) * S.T >= minDur) segs.push({ c0: s, c1: f - 1 }); s = -1; }
    }
    const kept = segs;
    const maxDur = P.maxlen / 1000;
    const maxFrames = Math.max(1, Math.round(maxDur / S.T));
    const split = [];
    for (const seg of kept) {
        const total = seg.c1 - seg.c0 + 1;
        if (total <= maxFrames) { split.push(seg); }
        else {
            for (let i = seg.c0; i <= seg.c1; i += maxFrames) {
                const end = Math.min(i + maxFrames - 1, seg.c1);
                split.push({ c0: i, c1: end });
            }
        }
    }
    segments = split.map(x => {
        const r = findPeaks(x.c0, x.c1, b0, b1);
        return {
            start: x.c0 * S.T, durMs: (x.c1 - x.c0 + 1) * S.T * 1000,
            c0: x.c0, c1: x.c1,
            peaks: r.peaks.map(p => ({ ...p, selected: true })),
            peakDb: r.peakDb, selected: true
        };
    });
    segT0 = segments.length ? segments[0].start : 0;
    gatedImg = makeSpectroImage($("cv").width, $("cv").height - MINI_H, true);
    cacheFullImages();
    analysisMs = performance.now() - t0;
    renderTable();
    render();
}

/* ---------------- rendering ---------------- */
function render() {
    const cv = $("cv"), ctx = cv.getContext("2d");
    if (!stft) return;
    const W = cv.width, H = cv.height;
    const S = stft;
    const useGated = !$("chkRaw").checked;
    const srcCanvas = useGated ? fullCanvasGated : fullCanvasRaw;
    if (!srcCanvas) return;

    const plotH = H - MINI_H; // main plot area (above minimap)

    /* --- vertical zoom: visible freq range from spectral gates, band fills ~80% --- */
    const { vLogMin, vLogMax, vSpan } = getVisibleLogRange();
    const fullLogMin = Math.log(HZ_MIN), fullLogMax = Math.log(HZ_MAX), fullSpan = fullLogMax - fullLogMin;
    const srcY0 = (vLogMin - fullLogMin) / fullSpan * plotH;
    const srcH = vSpan / fullSpan * plotH;
    const yF = hz => (Math.log(hz) - vLogMin) / vSpan * plotH;

    /* --- draw visible spectrogram slice (horizontal + vertical crop) --- */
    const t0px = viewStart / S.dur * W;
    const t1px = viewEnd / S.dur * W;
    ctx.fillStyle = "#0a0d11";
    ctx.fillRect(0, 0, W, H);
    // source Y is in the full-range spectrogram image; clamp to canvas
    const sy = clamp(srcY0, 0, plotH - 1), sh = clamp(srcH, 1, plotH - sy);
    ctx.drawImage(srcCanvas, t0px, sy, t1px - t0px, sh, 0, 0, W, plotH);
    ctx.font = "11px ui-monospace,monospace";
    for (const f of [100, 200, 500, 1000, 2000, 5000]) {
        if (f < HZ_MIN || f > HZ_MAX) continue;
        const y = yF(f);
        if (y < 0 || y > plotH) continue;
        ctx.globalAlpha = .18; ctx.strokeStyle = "#fff"; ctx.setLineDash([2, 4]);
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke(); ctx.setLineDash([]);
        ctx.globalAlpha = 1; ctx.fillStyle = "rgba(255,255,255,0.5)"; ctx.fillText(fmtHz(f), 6, y - 3);
    }

    /* --- time grid (adaptive) --- */
    const visDur = viewEnd - viewStart;
    let step = 1;
    const niceSteps = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 30, 60];
    for (const s of niceSteps) { if (s >= visDur / 12) { step = s; break; } }
    const tStart = Math.ceil(viewStart / step) * step;
    for (let t = tStart; t <= viewEnd; t += step) {
        const x = timeToX(t);
        ctx.globalAlpha = .18; ctx.strokeStyle = "#fff"; ctx.setLineDash([2, 4]);
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, plotH); ctx.stroke(); ctx.setLineDash([]);
        ctx.globalAlpha = 1; ctx.fillStyle = "rgba(255,255,255,0.5)";
        const lbl = step < 0.1 ? t.toFixed(2) : step < 1 ? t.toFixed(1) : t.toFixed(0);
        ctx.fillText(lbl + "s", x + 3, plotH - 5);
    }

    /* --- gate shading --- */
    ctx.fillStyle = "rgba(255,80,80,0.12)";
    if (P.fmin > HZ_MIN) ctx.fillRect(0, 0, W, yF(P.fmin));
    if (P.fmax < HZ_MAX) ctx.fillRect(0, yF(P.fmax), W, plotH - yF(P.fmax));
    ctx.strokeStyle = "rgba(255,120,120,0.55)"; ctx.setLineDash([6, 4]);
    ctx.beginPath(); ctx.moveTo(0, yF(P.fmin)); ctx.lineTo(W, yF(P.fmin)); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, yF(P.fmax)); ctx.lineTo(W, yF(P.fmax)); ctx.stroke(); ctx.setLineDash([]);
    const yn = (P.nlf + WINDB) / WINDB * plotH;
    ctx.strokeStyle = "rgba(255,210,80,0.7)"; ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(0, yn); ctx.lineTo(W, yn); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = "rgba(255,210,80,0.8)"; ctx.fillText("noise floor " + P.nlf + " dB", W - 160, yn - 5);

    /* --- segments (only visible ones) --- */
    ctx.lineWidth = 1.5;
    segments.forEach((sg, i) => {
        const sgEnd = sg.start + sg.durMs / 1000;
        if (sgEnd <= viewStart || sg.start >= viewEnd) return; // off-screen
        const x0 = timeToX(Math.max(sg.start, viewStart));
        const x1 = timeToX(Math.min(sgEnd, viewEnd));
        const y0 = yF(P.fmin), y1 = yF(P.fmax);
        if (sg.selected) {
            ctx.fillStyle = "rgba(0,255,255,0.07)"; ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
            ctx.strokeStyle = "rgba(0,255,255,0.8)"; ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
            ctx.fillStyle = "rgba(120,255,255,0.9)"; ctx.fillText(String(i + 1), x0 + 2, y0 + 12);
        } else {
            ctx.fillStyle = "rgba(120,120,120,0.05)"; ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
            ctx.strokeStyle = "rgba(150,150,150,0.5)"; ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
            ctx.fillStyle = "rgba(150,150,150,0.7)"; ctx.fillText(String(i + 1), x0 + 2, y0 + 12);
        }
    });

    /* --- minimap --- */
    drawMinimap(ctx, W, H, plotH);

    /* --- chime detector markers (when simulated) — view-relative, moves with zoom/pan --- */
    if (simResult) {
        for (const ev of simResult.events) {
            const t = ev.ms / 1000;
            if (t < viewStart || t > viewEnd) continue;
            const x = timeToX(t);
            if (x < 0 || x > W) continue;
            if (ev.type === "detected") {
                ctx.strokeStyle = "#3fd078"; ctx.lineWidth = 2;
                ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, plotH); ctx.stroke();
                ctx.lineWidth = 1;
                ctx.fillStyle = "#3fd078";
                ctx.fillText("✓ " + (ev.chimeName || "?"), x + 3, 12);
            } else if (/discount|timeout/.test(ev.type)) {
                ctx.strokeStyle = "rgba(255,92,92,0.7)"; ctx.setLineDash([4, 3]);
                ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, plotH); ctx.stroke();
                ctx.setLineDash([]);
            }
        }
    }

    /* --- status --- */
    const zoomed = (viewEnd - viewStart) < S.dur - 0.001;
    const zoomInfo = zoomed ? ` · view ${viewStart.toFixed(2)}–${viewEnd.toFixed(2)} s` : "";
    $("status").textContent = `frames ${stft.ncol} · ${stft.fs} Hz · ${segments.length} segment(s) (${segments.filter(s => s.selected).length} selected) · analysis ${analysisMs.toFixed(0)} ms${zoomInfo}`;
    scheduleSimChart();
}

function drawMinimap(ctx, W, H, plotH) {
    const my = plotH, mh = MINI_H;
    ctx.fillStyle = "#0a0d11";
    ctx.fillRect(0, my, W, mh);
    ctx.strokeStyle = "#262c34";
    ctx.strokeRect(0, my, W, mh);

    // downsampled spectrogram thumbnail
    const srcCanvas = $("chkRaw").checked ? fullCanvasRaw : fullCanvasGated;
    if (srcCanvas) {
        ctx.save();
        ctx.globalAlpha = 0.7;
        ctx.drawImage(srcCanvas, 0, 0, W, plotH, 0, my, W, mh);
        ctx.restore();
    }

    // viewport rectangle
    const vx0 = viewStart / stft.dur * W;
    const vx1 = viewEnd / stft.dur * W;
    ctx.fillStyle = "rgba(0,0,0,0.4)";
    ctx.fillRect(0, my, vx0, mh);
    ctx.fillRect(vx1, my, W - vx1, mh);
    ctx.strokeStyle = "var(--acc)";
    ctx.strokeStyle = "#3fd0ff";
    ctx.lineWidth = 1.5;
    ctx.strokeRect(vx0, my, vx1 - vx0, mh);
    ctx.lineWidth = 1;
}

function renderTable() {
    const tb = document.querySelector("#tbl tbody"); tb.innerHTML = "";
    $("empty").style.display = segments.length ? "none" : "block";
    $("empty").textContent = segments.length ? "" : "No segments detected — try lowering the noise floor or min length.";
    segments.forEach((sg, i) => {
        const tr = document.createElement("tr");
        if (!sg.selected) tr.className = "unsel";
        const peaksCell = sg.peaks.length
            ? sg.peaks.map((p, j) =>
                `<span class="pk${p.selected ? "" : " off"}" data-i="${i}" data-j="${j}">${p.f} Hz (${p.db.toFixed(1)} dB)</span>`
            ).join(' &nbsp;·&nbsp; ')
            : "—";
        tr.innerHTML = `<td class="mono"><input type="checkbox" class="sel" ${sg.selected ? "checked" : ""}></td>` +
            `<td class="mono">${i + 1}</td><td class="mono">${(sg.start - segT0).toFixed(2)}</td><td class="mono">${sg.durMs.toFixed(0)}</td>` +
            `<td class="peaks">${peaksCell}</td><td class="mono peak">${sg.peakDb.toFixed(1)}</td>` +
            `<td><button class="play" data-i="${i}">▶</button></td>`;
        tr.querySelectorAll(".pk").forEach(el => {
            el.onclick = e => {
                e.stopPropagation();
                const j = +el.dataset.j;
                sg.peaks[j].selected = !sg.peaks[j].selected;
                el.classList.toggle("off");
            };
        });
        tr.querySelector(".sel").onchange = e => { sg.selected = e.target.checked; tr.className = sg.selected ? "" : "unsel"; updateChkAll(); render(); };
        tr.querySelector(".play").onclick = () => playSegment(sg);
        tb.appendChild(tr);
    });
    updateChkAll();
}

function updateChkAll() {
    const all = $("chkAll");
    all.checked = segments.length > 0 && segments.every(s => s.selected);
    all.indeterminate = segments.some(s => s.selected) && !all.checked;
}

/* ---------------- playback ---------------- */
function newAudioCtx() { if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)(); return audioCtx; }
function buildBandpassGraph(ctx, src, freqs) {
    const sum = ctx.createGain();
    sum.gain.value = 1;
    for (const f of freqs) {
        const bp = ctx.createBiquadFilter();
        bp.type = "bandpass";
        bp.frequency.value = f;
        bp.Q.value = bpq;
        bp.detune.value = 0;
        src.connect(bp);
        bp.connect(sum);
    }
    sum.connect(ctx.destination);
    return src;
}
function playSegment(sg) {
    stopPlayback();
    const ctx = newAudioCtx(); if (ctx.resume) ctx.resume();
    const t0 = ctx.currentTime + 0.03;
    const startOffset = Math.max(0, sg.start);
    const dur = sg.durMs / 1000;
    const src = ctx.createBufferSource();
    src.buffer = fileBuf; src.connect(ctx.destination);
    playSrc = src;
    src.start(t0, startOffset);
    src.stop(t0 + dur + 0.05);
    src.onended = () => { playSrc = null; $("pos").textContent = ""; };
    $("pos").textContent = "playing seg " + (segments.indexOf(sg) + 1);
}
function playSelected() {
    stopPlayback();
    const sel = segments.filter(s => s.selected);
    if (!sel.length) { setMsg("No segments selected."); return; }
    setMsg("");
    const ctx = newAudioCtx(); if (ctx.resume) ctx.resume();
    const base = ctx.currentTime + 0.08;
    const anchor = sel[0].start;
    const arr = [];
    let skipped = 0;
    sel.forEach(sg => {
        const freqs = sg.peaks.filter(p => p.selected).map(p => p.f);
        if (!freqs.length) { skipped++; return; }
        const src = ctx.createBufferSource();
        src.buffer = fileBuf;
        buildBandpassGraph(ctx, src, freqs);
        const when = base + (sg.start - anchor);
        src.start(when, Math.max(0, sg.start));
        src.stop(when + sg.durMs / 1000 + 0.05);
        arr.push(src);
    });
    playSrc = arr;
    const total = (sel[sel.length - 1].start - anchor) + sel[sel.length - 1].durMs / 1000;
    $("pos").textContent = "playing " + sel.length + " selected · bandpass Q" + bpq + (skipped ? ` · ${skipped} skipped` : "");
    setTimeout(() => { if (playSrc === arr) { playSrc = null; $("pos").textContent = ""; } }, (total + 0.15) * 1000);
}
function playWhole() {
    stopPlayback();
    const ctx = newAudioCtx(); if (ctx.resume) ctx.resume();
    const src = ctx.createBufferSource();
    src.buffer = fileBuf; src.connect(ctx.destination);
    playSrc = src; src.start();
    src.onended = () => { playSrc = null; $("pos").textContent = ""; };
}
function stopPlayback() {
    const list = Array.isArray(playSrc) ? playSrc : (playSrc ? [playSrc] : []);
    list.forEach(s => { try { s.onended = null; s.stop(); } catch (e) { } });
    playSrc = null; $("pos").textContent = "";
}

/* ---------------- pipeline ---------------- */
function sizeCanvas() { const cv = $("cv"); cv.width = cv.clientWidth; cv.height = cv.clientHeight; }
function rebuildDSP(resetViewFlag) {
    const mono = toMono(fileBuf);
    const mono16 = scaleGain(resampleToTarget(mono, fileBuf.sampleRate, TARGET_FS));
    stft = buildSTFT(mono16, TARGET_FS);
    const plotH = $("cv").height - MINI_H;
    rawImg = makeSpectroImage($("cv").width, plotH, false);
    gatedImg = makeSpectroImage($("cv").width, plotH, true);
    cacheFullImages();
    if (resetViewFlag) resetView(); else clampView();
    analyze();
    if (simChimeList.length) runSim();
}
function runPipeline() {
    sizeCanvas();
    rebuildDSP(true);
    $("meta").textContent = `${fname} · ${fileBuf.duration.toFixed(2)} s · ${fileBuf.sampleRate} Hz src → ${TARGET_FS} Hz · ${fileBuf.numberOfChannels} ch`;
    $("btnPlay").disabled = false;
    setMsg("");
}

/* ---------------- load / decode ---------------- */
function setMsg(s) { $("msg").textContent = s || ""; }

function isPcmFile(f) { return f && /\.pcm$/i.test(f.name); }

function pcmBytesToAudioBuffer(int16) {
    // int16: Int16Array (16 kHz mono, signed LE)
    const ctx = newAudioCtx();
    const n = int16.length;
    if (!n) return ctx.createBuffer(1, 1, TARGET_FS);
    const buf = ctx.createBuffer(1, n, TARGET_FS);
    const ch = buf.getChannelData(0);
    for (let i = 0; i < n; i++) ch[i] = int16[i] / 32768;
    return buf;
}

function pcmArrayBufferToAudioBuffer(ab) {
    // ab: ArrayBuffer of raw 16-bit 16 kHz mono PCM (S16_LE)
    let bytes = ab.byteLength;
    if (bytes % 2) bytes -= 1; // drop trailing odd byte
    const n = bytes / 2;
    if (!n) {
        const ctx = newAudioCtx();
        return ctx.createBuffer(1, 1, TARGET_FS);
    }
    // Use DataView to decode LE explicitly (correct on both LE and BE hosts)
    const dv = new DataView(ab, 0, bytes);
    const int16 = new Int16Array(n);
    for (let i = 0; i < n; i++) int16[i] = dv.getInt16(i * 2, true);
    return pcmBytesToAudioBuffer(int16);
}

function onFile(f) {
    if (!f) return;
    if (micCapturing) { setMsg("Stop the mic capture first before loading a file."); return; }
    // Raw PCM path — no decodeAudioData (assumed 16-bit 16 kHz mono, matching mic_streamer)
    if (isPcmFile(f)) {
        setMsg("Loading raw PCM (16-bit 16 kHz mono)…");
        const reader = new FileReader();
        reader.onerror = () => setMsg("Could not read file: " + reader.error.message);
        reader.onload = () => {
            try {
                const ab = reader.result;
                if (!ab || !ab.byteLength) throw new Error("empty file");
                fileBuf = pcmArrayBufferToAudioBuffer(ab);
                fname = f.name; stopPlayback();
                setTimeout(runPipeline, 0);
            } catch (e) { setMsg("PCM load failed: " + e.message); fileBuf = null; }
        };
        reader.readAsArrayBuffer(f);
        return;
    }
    setMsg("Decoding & resampling…");
    const reader = new FileReader();
    reader.onerror = () => setMsg("Could not read file: " + reader.error.message);
    reader.onload = async () => {
        try {
            const ctx = newAudioCtx();
            fileBuf = await ctx.decodeAudioData(reader.result.slice(0));
        } catch (e) { setMsg("Decode failed — unsupported or corrupt format: " + e.message); fileBuf = null; return; }
        fname = f.name; stopPlayback();
        setTimeout(runPipeline, 0);
    };
    reader.readAsArrayBuffer(f);
}

/* ---------------- mic stream capture ---------------- */
function setMicUI() {
    const capturing = micCapturing;
    const urlInput = $("micUrl"), startBtn = $("btnMicStart"), stopBtn = $("btnMicStop");
    const fileInput = $("file"), testBtn = $("btnSimTest");
    if (urlInput) urlInput.disabled = capturing;
    if (startBtn) startBtn.disabled = capturing;
    if (stopBtn) stopBtn.disabled = !capturing;
    if (fileInput) fileInput.disabled = capturing;
    if (testBtn) testBtn.disabled = capturing;
}

function resetMicMeter() {
    const bar = $("micLevel"), wrap = $("micLvWrap"), meta = $("micMeta");
    if (bar) bar.style.width = "0%";
    if (wrap) { wrap.classList.remove("warn", "hot"); wrap.title = "Input level"; }
    if (meta && !micCapturing) meta.textContent = "idle";
    micPeakLin = 0;
}

function updateMicMeter() {
    const bar = $("micLevel"), wrap = $("micLvWrap"), meta = $("micMeta");
    if (!bar || !meta) return;
    const peak = micPeakLin;
    const peakDb = peak > 0 ? (20 * Math.log10(peak)).toFixed(1) : "-inf";
    const pct = peak > 0 ? clamp(((20 * Math.log10(peak) + 60) / 60) * 100, 0, 100) : 0;
    bar.style.width = pct.toFixed(1) + "%";
    if (wrap) {
        wrap.classList.remove("warn", "hot");
        if (pct > 92) wrap.classList.add("hot");
        else if (pct > 75) wrap.classList.add("warn");
        wrap.title = peakDb + " dBFS peak";
    }
    const secs = (micBytes / 2 / TARGET_FS).toFixed(2);
    if (micCapturing) {
        meta.innerHTML = '<span class="dot-rec"></span>' + secs + ' s &middot; ' + peakDb + ' dB';
    }
}

function resetMicState() {
    micCapturing = false;
    micAbortCtl = null;
    micReader = null;
    micChunks = [];
    micBytes = 0;
    if (micMeterTimer) { clearInterval(micMeterTimer); micMeterTimer = 0; }
    resetMicMeter();
    setMicUI();
}

async function startMicCapture() {
    if (micCapturing) return;
    const urlEl = $("micUrl");
    const rawUrl = urlEl ? urlEl.value.trim() : "";
    if (!rawUrl) { setMsg("Enter the mic_stream URL — e.g. http://doorbell.local/mic_stream"); if (urlEl) urlEl.focus(); return; }
    let parsed;
    try { parsed = new URL(rawUrl); if (parsed.protocol !== "http:" && parsed.protocol !== "https:") throw new Error("bad protocol"); }
    catch (e) { setMsg("Mic URL must be http:// or https:// — e.g. http://doorbell.local/mic_stream"); return; }
    const url = parsed.toString();
    if (urlEl) { try { localStorage.setItem("mic_stream_url", url); } catch (e) {} }

    setMsg("");
    micCapturing = true;
    micChunks = [];
    micBytes = 0;
    micPeakLin = 0;
    micAbortCtl = new AbortController();
    setMicUI();
    const meta = $("micMeta");
    if (meta) meta.innerHTML = '<span class="dot-rec"></span>connecting…';
    resetMicMeter();
    // live meter refresh ~12 fps
    micMeterTimer = setInterval(updateMicMeter, 80);

    let response;
    try {
        response = await fetch(url, { signal: micAbortCtl.signal, cache: "no-store", mode: "cors" });
    } catch (e) {
        if (e.name === "AbortError") { resetMicState(); return; }
        setMsg("Fetch failed — check the URL, device power, and that allow_without_auth: true is set. (" + (e.message || e) + ")");
        resetMicState();
        return;
    }
    if (!response.ok) {
        let hint = "";
        if (response.status === 429) hint = "Device is already streaming to another client (429). Stop the other capture and try again.";
        else if (response.status === 401 || response.status === 403) hint = "Authentication required (" + response.status + ") — set allow_without_auth: true on the mic_streamer.";
        else hint = "HTTP " + response.status + " " + response.statusText;
        setMsg("Mic stream request failed: " + hint);
        resetMicState();
        return;
    }
    const ct = (response.headers.get("content-type") || "").toLowerCase();
    if (ct && !ct.includes("audio/l16") && !ct.includes("octet-stream") && !ct.includes("application/octet-stream")) {
        // not fatal — some proxies strip the header; just warn in console
        console.warn("Unexpected mic_stream content-type:", ct);
    }
    if (!response.body || !response.body.getReader) {
        setMsg("This browser does not support streaming fetch (ReadableStream). Try a recent Chrome/Firefox.");
        resetMicState();
        return;
    }
    micReader = response.body.getReader();
    if (meta) meta.innerHTML = '<span class="dot-rec"></span>0.00 s &middot; capturing…';
    // For peak we must not drop a sample that straddles two fetch chunks (odd-length chunk)
    let micCarry = null; // pending low byte from previous odd-length chunk
    try {
        while (true) {
            const { value, done } = await micReader.read();
            if (done) break;
            if (!value || !value.length) continue;
            // track peak using DataView + carry (LE S16) so odd chunk boundaries don't corrupt
            let localPeak = 0;
            if (micCarry !== null) {
                // combine carry (low byte of next sample) with first byte of this chunk (high byte)
                if (value.length >= 1) {
                    const first = (value[0] << 8) | micCarry;
                    // sign-extend 16-bit
                    const s = (first << 16) >> 16;
                    const v = Math.abs(s) / 32768;
                    if (v > localPeak) localPeak = v;
                    // decoded one byte from value; remaining bytes start at offset 1
                    const rem = value.length - 1;
                    const n16rem = rem >> 1;
                    if (n16rem) {
                        const dv = new DataView(value.buffer, value.byteOffset + 1, n16rem * 2);
                        const step = n16rem > 2048 ? 4 : 1;
                        for (let i = 0; i < n16rem; i += step) {
                            const s2 = dv.getInt16(i * 2, true);
                            const v2 = Math.abs(s2) / 32768;
                            if (v2 > localPeak) localPeak = v2;
                        }
                    }
                    micCarry = (rem % 2) ? value[value.length - 1] : null;
                } else {
                    // value empty but we still have carry — keep it
                }
            } else {
                const n16 = value.length >> 1;
                if (n16) {
                    const dv = new DataView(value.buffer, value.byteOffset, n16 * 2);
                    const step = n16 > 2048 ? 4 : 1;
                    for (let i = 0; i < n16; i += step) {
                        const s = dv.getInt16(i * 2, true);
                        const v = Math.abs(s) / 32768;
                        if (v > localPeak) localPeak = v;
                    }
                }
                micCarry = (value.length % 2) ? value[value.length - 1] : null;
            }
            if (localPeak > micPeakLin) micPeakLin = localPeak;
            // store copy (value will be detached after next read on some impls)
            micChunks.push(value.slice(0));
            micBytes += value.length;
            // throttle DOM update — timer does it, but also push immediate for first data
        }
    } catch (e) {
        if (e.name === "AbortError") {
            // user pressed stop — fall through to finalize
        } else {
            console.error("mic stream read error", e);
            setMsg("Stream ended with error: " + (e.message || e));
            // still finalize what we have
        }
    } finally {
        // if still marked capturing, this was a natural end (device closed at max_duration) or an error — finalize
        // if user pressed Stop & analyse, stopMicCapture already aborted and will call finalize; avoid double-finalize
        if (micCapturing) {
            finalizeMicCapture();
        }
    }
}

function stopMicCapture() {
    if (!micCapturing) return;
    // abort fetch; read loop will exit and finalize via its finally block
    // if fetch already done, just finalize now
    const ctl = micAbortCtl;
    if (ctl) { try { ctl.abort(); } catch (e) {} }
    else { finalizeMicCapture(); }
    // UI feedback immediately
    const meta = $("micMeta");
    if (meta) meta.textContent = "stopping…";
}

function finalizeMicCapture() {
    const chunks = micChunks.slice();
    const totalBytes = micBytes;
    const peak = micPeakLin;
    const wasCapturing = micCapturing;
    // reset state before pipeline (so file input is re-enabled)
    resetMicState();
    if (!wasCapturing && !chunks.length) return;
    if (!totalBytes) { setMsg("No audio captured — stream was empty. Check the device microphone."); return; }
    // drop trailing odd byte (global, not per-chunk) and decode LE correctly
    let bytes = totalBytes;
    if (bytes % 2) bytes -= 1;
    const samples = bytes / 2;
    if (samples < TARGET_FS * 0.08) { // < 80 ms
        setMsg("Capture too short (" + (samples / TARGET_FS).toFixed(2) + " s) — hold the chime longer and try again.");
        return;
    }
    // concat raw bytes first, then decode S16_LE with DataView so a sample
    // straddling two fetch chunks is preserved (old code dropped per-chunk odd byte)
    const concat = new Uint8Array(bytes);
    let off = 0;
    for (const c of chunks) {
        if (off >= bytes) break;
        const take = Math.min(c.length, bytes - off);
        if (take <= 0) break;
        concat.set(c.subarray(0, take), off);
        off += take;
    }
    const out = new Int16Array(samples);
    const dv = new DataView(concat.buffer, concat.byteOffset, bytes);
    for (let i = 0; i < samples; i++) out[i] = dv.getInt16(i * 2, true);
    try {
        fileBuf = pcmBytesToAudioBuffer(out);
    } catch (e) { setMsg("Failed to build audio buffer: " + e.message); return; }
    let host = "mic stream";
    try { const u = new URL($("micUrl") ? $("micUrl").value.trim() : ""); host = u.hostname || "mic stream"; } catch (e) {}
    const secs = (samples / TARGET_FS).toFixed(2);
    const peakDb = peak > 0 ? (20 * Math.log10(peak)).toFixed(1) : "-inf";
    fname = host + " · mic stream " + secs + "s";
    stopPlayback();
    setMsg("Captured " + secs + " s (" + samples + " samples, peak " + peakDb + " dBFS) — analysing…");
    setTimeout(() => { runPipeline(); setTimeout(() => setMsg(""), 2500); }, 0);
}

/* ---------------- controls ---------------- */
const PARAMS = [
    { key: "nlf", r: "r_nlf", n: "n_nlf", min: -80, max: -10 },
    { key: "minlen", r: "r_minlen", n: "n_minlen", min: 10, max: 500 },
    { key: "maxlen", r: "r_maxlen", n: "n_maxlen", min: 100, max: 5000 },
    { key: "fmin", r: "r_fmin", n: "n_fmin", min: 50, max: 8000 },
    { key: "fmax", r: "r_fmax", n: "n_fmax", min: 500, max: 8000 },
    { key: "topn", r: "r_topn", n: "n_topn", min: 1, max: 10 },
];
let rafPending = false;
function onParamChange() {
    if (!stft) return;
    if (!rafPending) { rafPending = true; requestAnimationFrame(() => { rafPending = false; analyze(); }); }
}
PARAMS.forEach(p => {
    const r = $(p.r), n = $(p.n);
    const apply = v => { v = clamp(+v || DEFAULTS[p.key], p.min, p.max); P[p.key] = v; r.value = v; n.value = v; onParamChange(); };
    r.addEventListener("input", () => apply(r.value));
    n.addEventListener("change", () => apply(n.value));
});
$("btnReset").onclick = () => { Object.assign(P, DEFAULTS); PARAMS.forEach(p => { $(p.r).value = P[p.key]; $(p.n).value = P[p.key]; }); bpq = BPQ_DEFAULT; $("r_bpq").value = bpq; $("n_bpq").value = bpq; onParamChange(); };
$("btnPlaySel").onclick = playSelected;
$("chkAll").onchange = e => { segments.forEach(s => s.selected = e.target.checked); renderTable(); render(); };
$("chkRaw").onchange = render;
$("btnPlay").onclick = () => { if (fileBuf) playWhole(); };
$("btnStop").onclick = stopPlayback;
$("file").addEventListener("change", e => { onFile(e.target.files[0]); e.target.value = ""; });
$("r_bpq").value = bpq; $("n_bpq").value = bpq;
$("r_bpq").addEventListener("input", () => { bpq = clamp(+$("r_bpq").value || BPQ_DEFAULT, 1, 30); $("n_bpq").value = bpq; });
$("n_bpq").addEventListener("change", () => { bpq = clamp(+$("n_bpq").value || BPQ_DEFAULT, 1, 30); $("r_bpq").value = bpq; });
/* input gain: full DSP rebuild (STFT + images + analysis + sim), debounced */
let gainTimer = 0;
function setGainDb(v) {
    gainDb = clamp(+v || 0, -48, 24);
    $("r_gain").value = gainDb; $("n_gain").value = gainDb;
    if (!stft) return; // nothing loaded yet — applies on next runPipeline
    clearTimeout(gainTimer);
    gainTimer = setTimeout(() => rebuildDSP(false), 120);
}
$("r_gain").addEventListener("input", () => setGainDb($("r_gain").value));
$("n_gain").addEventListener("change", () => setGainDb($("n_gain").value));
/* mic stream capture wiring */
try { const saved = localStorage.getItem("mic_stream_url"); if (saved && $("micUrl")) $("micUrl").value = saved; } catch (e) {}
if ($("btnMicStart")) $("btnMicStart").onclick = startMicCapture;
if ($("btnMicStop")) $("btnMicStop").onclick = stopMicCapture;
if ($("micUrl")) $("micUrl").addEventListener("keydown", e => { if (e.key === "Enter") { e.preventDefault(); startMicCapture(); } });
setMicUI(); resetMicMeter();

/* ---- zoom & pan event handlers ---- */

/* mouse wheel → zoom */
$("cv").addEventListener("wheel", e => {
    if (!stft) return;
    e.preventDefault();
    const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15;
    const t = canvasTime(e.clientX);
    const newDur = (viewEnd - viewStart) / factor;
    const ratio = (t - viewStart) / (viewEnd - viewStart);
    viewStart = t - ratio * newDur;
    viewEnd = viewStart + newDur;
    clampView();
    render();
}, { passive: false });

/* mousedown / mousemove / mouseup → pan (with click threshold) */
$("cv").addEventListener("mousedown", e => {
    if (!stft) return;
    const plotH = $("cv").height - MINI_H;
    const rect = $("cv").getBoundingClientRect();
    const my = (e.clientY - rect.top) * ($("cv").height / rect.height);

    if (my >= plotH) {
        /* clicked in minimap area → jump */
        const mx = (e.clientX - rect.left) * ($("cv").width / rect.width);
        const dur = viewEnd - viewStart;
        const t = (mx / $("cv").width) * stft.dur;
        viewStart = clamp(t - dur / 2, 0, Math.max(0, stft.dur - dur));
        viewEnd = viewStart + dur;
        clampView();
        miniDrag = true;
        miniDragX = mx;
        miniDragVS = viewStart;
        miniDragVE = viewEnd;
        $("cv").style.cursor = "grabbing";
        return;
    }

    /* main plot area */
    isPanning = false;
    panStartX = e.clientX;
    panStartVS = viewStart;
    panStartVE = viewEnd;
    panMoved = false;
    $("cv").style.cursor = "grabbing";
});

window.addEventListener("mousemove", e => {
    if (!stft) return;
    const cv = $("cv"), rect = cv.getBoundingClientRect();
    const plotH = cv.height - MINI_H;
    const mx = (e.clientX - rect.left) * (cv.width / rect.width);
    const my = (e.clientY - rect.top) * (cv.height / rect.height);

    if (miniDrag) {
        const dx = mx - miniDragX;
        const dt = dx / cv.width * stft.dur;
        viewStart = miniDragVS + dt;
        viewEnd = miniDragVE + dt;
        clampView();
        render();
        return;
    }

    if (!stft || (isPanning || panMoved)) {
        if (panStartX !== 0 || isPanning) {
            const dx = e.clientX - panStartX;
            if (Math.abs(dx) > 3) panMoved = true;
            if (panMoved) {
                isPanning = true;
                const dt = -dx / cv.width * (panStartVE - panStartVS);
                viewStart = panStartVS + dt;
                viewEnd = panStartVE + dt;
                clampView();
                render();
            }
        }
        return;
    }

    /* hover readout (not panning) — hz uses same log mapping as yF for exact alignment */
    if (my < plotH) {
        const S = stft, { vLogMin, vSpan } = getVisibleLogRange();
        const hz = Math.exp(vLogMin + my / plotH * vSpan), t = viewStart + mx / cv.width * (viewEnd - viewStart);
        const f = Math.min(S.ncol - 1, Math.round(t / S.T)), k = clamp(Math.round(hz / S.db), 0, NFFT / 2 - 1);
        $("readout").textContent = `t = ${t.toFixed(3)} s   f = ${hz.toFixed(1)} Hz   mag = ${(10 * Math.log10(S.frames[f][k] * S.frames[f][k] + 1e-30)).toFixed(1)} dB`;
    }
});

window.addEventListener("mouseup", e => {
    const wasPanning = isPanning || panMoved;
    isPanning = false;
    panMoved = false;
    miniDrag = false;
    $("cv").style.cursor = "crosshair";

    if (!stft) return;

    /* if it was a click (not a drag), toggle segment selection */
    if (!wasPanning) {
        const cv = $("cv"), rect = cv.getBoundingClientRect();
        const plotH = cv.height - MINI_H;
        const mx = (e.clientX - rect.left) * (cv.width / rect.width);
        const my = (e.clientY - rect.top) * (cv.height / rect.height);
        if (my < plotH) {
            const S = stft;
            const t = viewStart + mx / cv.width * (viewEnd - viewStart);
            const { vLogMin, vSpan } = getVisibleLogRange();
            const hz = Math.exp(vLogMin + my / plotH * vSpan);
            if (hz < P.fmin || hz > P.fmax) return;
            for (let i = segments.length - 1; i >= 0; i--) {
                const sg = segments[i];
                if (t >= sg.start && t < sg.start + sg.durMs / 1000) {
                    sg.selected = !sg.selected;
                    renderTable();
                    render();
                    return;
                }
            }
        }
    }
});

/* double-click → reset view */
$("cv").addEventListener("dblclick", e => {
    if (!stft) return;
    resetView();
    render();
});

/* leave canvas → clear readout */
$("cv").addEventListener("mouseleave", () => $("readout").textContent = "");

/* keyboard zoom/pan */
document.addEventListener("keydown", e => {
    if (!stft) return;
    /* don't intercept when typing in inputs */
    if (e.target.tagName === "INPUT" || e.target.tagName === "TEXTAREA") return;
    const dur = viewEnd - viewStart;
    const panStep = dur * 0.15; // 15% of visible window
    switch (e.key) {
        case "+": case "=":
            e.preventDefault();
            { const c = (viewStart + viewEnd) / 2; const nd = dur / 1.3; viewStart = c - nd / 2; viewEnd = c + nd / 2; clampView(); render(); }
            break;
        case "-": case "_":
            e.preventDefault();
            { const c = (viewStart + viewEnd) / 2; const nd = Math.min(dur * 1.3, stft.dur); viewStart = c - nd / 2; viewEnd = c + nd / 2; clampView(); render(); }
            break;
        case "ArrowLeft":
            e.preventDefault();
            viewStart -= panStep; viewEnd -= panStep; clampView(); render();
            break;
        case "ArrowRight":
            e.preventDefault();
            viewStart += panStep; viewEnd += panStep; clampView(); render();
            break;
        case "Home":
            e.preventDefault();
            viewStart = 0; viewEnd = dur; clampView(); render();
            break;
        case "End":
            e.preventDefault();
            viewStart = stft.dur - dur; viewEnd = stft.dur; clampView(); render();
            break;
        case "0":
            e.preventDefault();
            resetView(); render();
            break;
    }
});


/* ============================================================
   chime detector simulator — WASM-only
   Uses esphome/components/chime/core compiled to WASM via
   util/ (emsdk 3.1.6) and served as wasm/chime.js + wasm/chime.wasm.
   All DSP runs in C++ (ChimeEngine::run_offline); no JS fallback.
   ============================================================ */

// WASM module — MODULARIZE=1 factory (wasm/chime.js)
let ChimeWasmModule = null;
let wasmReady = false;
let wasmLoadError = null;
let wasmReadyPromise = null;
if (typeof ChimeWasm !== 'undefined') {
  wasmReadyPromise = ChimeWasm().then(function(m) {
    ChimeWasmModule = m;
    wasmReady = true;
    const btn = document.getElementById('btnSimRun');
    if (btn) btn.disabled = false;
    const badge = document.getElementById('simStatus');
    if (badge) {
      const cur = badge.textContent || '';
      badge.textContent = (cur ? cur + ' · ' : '') + 'WASM ready (core/)';
    }
    if (stft) scheduleSimRun();
    return m;
  }).catch(function(e) {
    wasmLoadError = e;
    console.error('ChimeWasm load failed', e);
    const badge = document.getElementById('simStatus');
    if (badge) badge.textContent = 'WASM load failed: ' + (e && e.message ? e.message : e);
    const btn = document.getElementById('btnSimRun');
    if (btn) btn.disabled = true;
  });
} else {
  console.error('ChimeWasm factory not found — ensure <script src="wasm/chime.js"></script> loads before app.js');
  wasmLoadError = new Error('ChimeWasm not found');
}

let simChimeList = [];   // editable per-chime configs
let simResult = null;    // ChimeDetector.run() output
let simTickIdx = 0;      // tick shown in the diagnostics table
let simDebounce = 0;

const SIM_DEF = { windowSize: 1024, tickIntervalMs: 100, guardHz: 150, alphaDown: 0.05, alphaUp: 0.005 };
const SIM_COLORS = ["#3fd0ff", "#ffb454", "#c084fc", "#3fd078", "#ff7eb6", "#7ee787", "#f778ba", "#67e8f9"];

function parseFreqs(s) {
    const out = [];
    (s || "").split(/[,;\s]+/).forEach(x => {
        const v = parseFloat(x);
        if (isFinite(v) && v > 0) out.push(v);
    });
    return out;
}
function simNum(v, d) { const x = +v; return isFinite(x) ? x : d; }

function addSimChime(spec) {
    simChimeList.push(Object.assign({
        enabled: true,
        name: "chime_" + (simChimeList.length + 1),
        minMs: 2000, maxMs: 8000,
        thr: -50, snr: 8, prom: 0, onset: 8, tail: 2000,
        steps: [{ chord: "440", time: "" }, { chord: "660", time: "0.5" }]
    }, spec || {}));
}

const SIM_HINTS = {
    minMs: "Pattern must last at least this long (too-fast matches rejected).",
    maxMs: "Pattern must finish within this time. Sensor release = max + 2 s.",
    thr: "Minimum level a tone must exceed to count as present.",
    snr: "How far above the adaptive noise floor the tone must sit.",
    prom: "How far the tone must exceed local background (guard band). Uses other bins — ineffective with a single distinct frequency (leave at 0).",
    onset: "How far the tone must exceed its own level 5 ticks earlier (0 = off).",
    tail: "Extra time allowed after a timed step before it is a miss."
};

function simNumCtl(k, label, val, unit) {
    const hint = SIM_HINTS[k] || "";
    return `<div class="ctl"><div class="lab"><span title="${hint}">${label}</span><i><input class="n_${k}" type="number" value="${val}" step="1"> ${unit}</i></div>${hint ? `<div class="hint">${hint}</div>` : ""}</div>`;
}

function simChimeCard(c, i) {
    const el = document.createElement("div");
    el.className = "chime-card";
    el.innerHTML =
        `<div class="cc-top">
            <label><input type="checkbox" class="cc-on" ${c.enabled ? "checked" : ""}> enabled</label>
            <input class="cc-name" type="text" value="${c.name.replace(/"/g, "&quot;")}" spellcheck="false">
            <span class="meta">release timeout = max + 2000 ms</span>
            <button class="cc-del">remove</button>
        </div>
        <div class="pgrid">
            ${simNumCtl("minMs", "Min duration", c.minMs, "ms")}
            ${simNumCtl("maxMs", "Max duration", c.maxMs, "ms")}
            ${simNumCtl("thr", "Threshold", c.thr, "dB")}
            ${simNumCtl("snr", "SNR margin", c.snr, "dB")}
            ${simNumCtl("prom", "Prominence", c.prom, "dB")}
            ${simNumCtl("onset", "Onset contrast", c.onset, "dB")}
            ${simNumCtl("tail", "Tail grace", c.tail, "ms")}
        </div>
        <table class="pat">
            <thead><tr><th>#</th><th>chord (Hz, comma-separated)</th><th>time (s · blank = any time)</th><th></th></tr></thead>
            <tbody></tbody>
        </table>
        <div class="btns"><button class="pt-add">+ step</button></div>`;
    const tbody = el.querySelector("tbody");
    c.steps.forEach((s, si) => {
        const tr = document.createElement("tr");
        tr.innerHTML = `<td>${si + 1}</td><td><input class="pt-chord" type="text" value="${s.chord.replace(/"/g, "&quot;")}" spellcheck="false"></td><td><input class="pt-time" type="text" value="${s.time.replace(/"/g, "&quot;")}" style="width:90px" spellcheck="false"></td><td><button class="pt-del" title="remove step">×</button></td>`;
        tr.querySelector(".pt-del").onclick = () => { if (c.steps.length > 1) { c.steps.splice(si, 1); buildSimChimes(); } };
        tbody.appendChild(tr);
    });
    el.querySelector(".pt-add").onclick = () => { c.steps.push({ chord: "880", time: "" }); buildSimChimes(); };
    el.querySelector(".cc-del").onclick = () => { simChimeList.splice(i, 1); buildSimChimes(); };
    ["minMs", "maxMs", "thr", "snr", "prom", "onset", "tail"].forEach(k => {
        const inp = el.querySelector(".n_" + k);
        inp.addEventListener("change", () => { c[k] = simNum(inp.value, c[k]); scheduleSimRun(); });
    });
    el.querySelector(".cc-name").addEventListener("change", e => { c.name = e.target.value.trim() || c.name; scheduleSimRun(); });
    el.querySelector(".cc-on").addEventListener("change", e => { c.enabled = e.target.checked; scheduleSimRun(); });
    tbody.querySelectorAll("tr").forEach((tr, si) => {
        tr.querySelector(".pt-chord").addEventListener("change", e => { c.steps[si].chord = e.target.value; scheduleSimRun(); });
        tr.querySelector(".pt-time").addEventListener("change", e => { c.steps[si].time = e.target.value; scheduleSimRun(); });
    });
    return el;
}

function buildSimChimes() {
    const host = $("simChimes");
    host.innerHTML = "";
    simChimeList.forEach((c, i) => host.appendChild(simChimeCard(c, i)));
    renderSimResults();
    renderSimYaml();
}

function simSharedCfg() {
    const v = id => +$(id).value;
    return {
        windowSize: clamp(Math.round(v("simWin") || SIM_DEF.windowSize), 64, 4096),
        tickIntervalMs: clamp(Math.round(v("simTick") || SIM_DEF.tickIntervalMs), 10, 500),
        guardSeparationHz: clamp(v("simGuard") || SIM_DEF.guardHz, 10, 1000),
        noiseFloorAlphaDown: clamp(v("simAd") || SIM_DEF.alphaDown, 0.0001, 1),
        noiseFloorAlphaUp: clamp(v("simAu") || SIM_DEF.alphaUp, 0.0001, 1)
    };
}

function readSimChimesForWasm() {
    const out = [];
    for (const c of simChimeList) {
        if (!c.enabled) continue;
        const pattern = c.steps.map(s => parseFreqs(s.chord));
        if (!pattern.length || pattern.some(p => !p.length)) continue;
        out.push({
            name: c.name || "chime",
            pattern: pattern,
            times: c.steps.map(s => {
                const t = (s.time || "").trim();
                if (!t) return null;
                const v = parseFloat(t);
                return isFinite(v) ? v : null;
            }),
            minDurationMs: Math.max(0, simNum(c.minMs, 0)),
            maxDurationMs: Math.max(100, simNum(c.maxMs, 8000)),
            thresholdDb: simNum(c.thr, -50),
            snrMarginDb: simNum(c.snr, 0),
            prominenceDb: simNum(c.prom, 0),
            onsetContrastDb: simNum(c.onset, 0),
            tailGraceMs: Math.max(100, simNum(c.tail, 2000))
        });
    }
    return out;
}

function runSim() {
    if (!stft) return;
    if (!wasmReady || !ChimeWasmModule || !ChimeWasmModule.runChimeDetector) {
        if (wasmLoadError) {
            $("simStatus").textContent = "WASM not ready: " + (wasmLoadError.message || wasmLoadError);
        } else {
            $("simStatus").textContent = "WASM loading…";
            if (wasmReadyPromise) wasmReadyPromise.then(() => runSim());
        }
        return;
    }
    const shared = simSharedCfg();
    const chimes = readSimChimesForWasm();
    if (!chimes.length) {
        simResult = null;
        renderSimResults();
        render();
        $("simStatus").textContent = "no enabled chimes with valid chords";
        return;
    }
    const t0 = performance.now();
    const opts = Object.assign({ chimes: chimes }, shared);
    let raw;
    try {
        raw = ChimeWasmModule.runChimeDetector(stft.mono, stft.fs, opts);
    } catch (e) {
        console.error("WASM runChimeDetector failed", e);
        $("simStatus").textContent = "WASM run failed: " + (e && e.message ? e.message : e);
        return;
    }
    // Normalize to shape expected by renderers (js/chime.js compat)
    // Ensure events have chimeName
    if (raw.events) {
        raw.events.forEach(function(ev){
            if (ev.chimeName == null && ev.chime != null && chimes[ev.chime]) ev.chimeName = chimes[ev.chime].name;
        });
    }
    // Build chimes array with thresholdDb and sensorIntervals derived from events
    const wasmChimes = chimes.map(function(c){ return { name: c.name, thresholdDb: c.thresholdDb, sensorIntervals: [] }; });
    if (raw.events) {
        raw.events.forEach(function(ev){
            if (ev.type === "detected") {
                const idx = ev.chime;
                if (wasmChimes[idx]) wasmChimes[idx].sensorIntervals.push({ startMs: ev.ms, endMs: null });
            } else if (ev.type === "released") {
                const idx = ev.chime;
                const arr = wasmChimes[idx] ? wasmChimes[idx].sensorIntervals : null;
                if (arr && arr.length) {
                    const last = arr[arr.length-1];
                    if (last && last.endMs == null) last.endMs = ev.ms;
                }
            }
        });
        const durMs = raw.meta ? raw.meta.durationMs : 0;
        wasmChimes.forEach(function(c){
            c.sensorIntervals.forEach(function(iv){ if (iv.endMs == null) iv.endMs = durMs; });
        });
    }
    raw.chimes = wasmChimes;
    // Ensure each tick has .chimes snapshot with fields renderSimDiag expects
    if (raw.ticks) {
        raw.ticks.forEach(function(tk){
            if (!tk.chimes || tk.chimes.length !== wasmChimes.length) {
                // synthesize from diag
                tk.chimes = tk.diag.map(function(d){
                    return {
                        name: d.name,
                        active: d.active,
                        matchIndex: d.step,
                        numSteps: d.numSteps,
                        needFallingEdge: false,
                        latched: false,
                        elapsedMs: 0
                    };
                });
            } else {
                // enrich existing entries with defaults for missing fields
                tk.chimes.forEach(function(cs){
                    if (cs.needFallingEdge == null) cs.needFallingEdge = false;
                    if (cs.latched == null) cs.latched = false;
                    if (cs.elapsedMs == null) cs.elapsedMs = 0;
                    if (cs.matchIndex == null) cs.matchIndex = 0;
                    if (cs.numSteps == null) cs.numSteps = 0;
                });
            }
        });
    }
    simResult = raw;
    simTickIdx = Math.max(0, simResult.ticks.length - 1);
    $("simStatus").textContent = `WASM · ${simResult.meta.totalTicks} ticks \u00d7 ${simResult.meta.tickAudioMs} ms \u00b7 ${simResult.events.length} events \u00b7 ${(performance.now() - t0).toFixed(0)} ms`;
    renderSimResults();
    render();
}

function scheduleSimRun() {
    renderSimYaml();
    if (!stft) return;
    clearTimeout(simDebounce);
    simDebounce = setTimeout(runSim, 300);
}

/* ---------- simulator rendering ---------- */

function renderSimResults() {
    const strip = $("simStrip"), chart = $("simChart");
    strip.width = strip.clientWidth;
    chart.width = chart.clientWidth;
    strip.getContext("2d").clearRect(0, 0, strip.width, strip.height);
    chart.getContext("2d").clearRect(0, 0, chart.width, chart.height);
    if (!simResult) {
        $("simEvents").innerHTML = '<div class="empty">No results yet — load audio (or “generate test signal”) and press “run detection”.</div>';
        $("simDiag").innerHTML = "";
        $("simDiagTick").textContent = "–";
        return;
    }
    renderSimStrip();
    renderSimChart();
    renderSimEvents();
    renderSimDiag();
}

function renderSimStrip() {
    const cv = $("simStrip"), c2 = cv.getContext("2d");
    const rows = simResult.chimes.length;
    cv.height = rows * 18 + 10;
    const dur = Math.max(simResult.meta.durationMs / 1000, 0.001);
    const X = t => 96 + (t / dur) * (cv.width - 100);
    c2.clearRect(0, 0, cv.width, cv.height);
    c2.strokeStyle = "#1d232b"; c2.lineWidth = 1;
    for (let i = 1; i < 10; i++) {
        const x = 96 + i / 10 * (cv.width - 100);
        c2.beginPath(); c2.moveTo(x, 0); c2.lineTo(x, cv.height); c2.stroke();
    }
    simResult.chimes.forEach((c, i) => {
        const y = 6 + i * 18;
        c2.fillStyle = "#8a94a0"; c2.font = "10px ui-monospace,monospace";
        c2.fillText(c.name.slice(0, 12), 4, y + 9);
        for (const iv of c.sensorIntervals) {
            const x0 = X(iv.startMs / 1000), x1 = X(iv.endMs / 1000);
            c2.fillStyle = "rgba(63,208,120,0.28)";
            c2.fillRect(x0, y, Math.max(2, x1 - x0), 12);
            c2.fillStyle = "#3fd078";
            c2.fillRect(x0, y, 2, 12);
        }
    });
}

let simChartRaf = 0;
function scheduleSimChart() {
    if (simChartRaf) return;
    simChartRaf = requestAnimationFrame(() => { simChartRaf = 0; if (simResult && stft) renderSimChart(); });
}

function renderSimChart() {
    const cv = $("simChart"), c2 = cv.getContext("2d");
    const W = cv.width, H = cv.height;
    const padT = 8, padB = 20;
    c2.clearRect(0, 0, W, H);
    if (!simResult || !stft || !W) return;
    const t0 = viewStart, t1 = viewEnd;
    const X = t => (t - t0) / (t1 - t0) * W; // same mapping as the spectrogram
    const ticks = simResult.ticks;
    let lo = Infinity, hi = -Infinity;
    for (const tk of ticks) {
        const t = tk.ms / 1000;
        if (t < t0 || t > t1) continue;
        for (const b of tk.bins) {
            if (b.db < lo) lo = b.db;
            if (b.db > hi) hi = b.db;
            if (b.floor != null) {
                if (b.floor < lo) lo = b.floor;
                if (b.floor > hi) hi = b.floor;
            }
        }
    }
    if (lo === Infinity) { lo = -80; hi = 0; }
    if (hi - lo < 20) { const m = (hi + lo) / 2; lo = m - 10; hi = m + 10; }
    const Y = db => H - padB - (db - lo) / (hi - lo) * (H - padT - padB);
    c2.font = "10px ui-monospace,monospace";
    for (let g = 0; g <= 4; g++) {
        const db = lo + (hi - lo) * g / 4;
        const y = Y(db);
        c2.strokeStyle = "#1d232b";
        c2.beginPath(); c2.moveTo(0, y); c2.lineTo(W, y); c2.stroke();
        c2.fillStyle = "#8a94a0";
        c2.fillText(db.toFixed(0) + " dB", 4, y - 3);
    }
    // time grid — identical adaptive step to the spectrogram so vertical lines align
    const visDur = t1 - t0;
    let step = 1;
    const niceSteps = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 30, 60];
    for (const s of niceSteps) { if (s >= visDur / 12) { step = s; break; } }
    const tStart = Math.ceil(t0 / step) * step;
    for (let t = tStart; t <= t1; t += step) {
        const x = X(t);
        c2.strokeStyle = "rgba(255,255,255,0.14)";
        c2.setLineDash([2, 4]);
        c2.beginPath(); c2.moveTo(x, padT); c2.lineTo(x, H - padB); c2.stroke();
        c2.setLineDash([]);
        c2.fillStyle = "#8a94a0";
        const lbl = step < 0.1 ? t.toFixed(2) : step < 1 ? t.toFixed(1) : t.toFixed(0);
        c2.fillText(lbl + "s", x + 3, H - 6);
    }
    // Ticks inside the view (decimated when dense), computed once.
    let viewTicks = [];
    for (const tk of ticks) {
        const t = tk.ms / 1000;
        if (t >= t0 - 0.05 && t <= t1 + 0.05) viewTicks.push(tk);
    }
    if (viewTicks.length > 6000) {
        const k = Math.ceil(viewTicks.length / 6000);
        const thinned = [];
        for (let i = 0; i < viewTicks.length; i += k) thinned.push(viewTicks[i]);
        viewTicks = thinned;
    }
    const nf = simResult.meta.bins;
    for (let b = 0; b < nf; b++) {
        const col = SIM_COLORS[b % SIM_COLORS.length];
        // spectrum level
        c2.strokeStyle = col; c2.globalAlpha = 0.9; c2.lineWidth = 1.2;
        c2.beginPath();
        let started = false;
        for (const tk of viewTicks) {
            const y = Y(tk.bins[b].db);
            if (!started) { c2.moveTo(X(tk.ms / 1000), y); started = true; } else c2.lineTo(X(tk.ms / 1000), y);
        }
        c2.stroke();
        // adaptive noise floor (dashed)
        c2.setLineDash([3, 3]); c2.globalAlpha = 0.55; c2.lineWidth = 1;
        c2.beginPath(); started = false;
        for (const tk of viewTicks) {
            if (tk.bins[b].floor == null) { started = false; continue; }
            const y = Y(tk.bins[b].floor);
            if (!started) { c2.moveTo(X(tk.ms / 1000), y); started = true; } else c2.lineTo(X(tk.ms / 1000), y);
        }
        c2.stroke();
        c2.setLineDash([]);
    }
    c2.globalAlpha = 1;
    // per-chime effective threshold lines
    simResult.chimes.forEach((c, i) => {
        const col = SIM_COLORS[i % SIM_COLORS.length];
        const y = Y(c.thresholdDb);
        if (y < padT || y > H - padB) return;
        c2.strokeStyle = col; c2.globalAlpha = 0.5; c2.setLineDash([6, 4]); c2.lineWidth = 1;
        c2.beginPath(); c2.moveTo(0, y); c2.lineTo(W, y); c2.stroke();
        c2.setLineDash([]); c2.globalAlpha = 0.9;
        c2.fillStyle = col;
        c2.fillText(c.name.slice(0, 14) + " thr " + c.thresholdDb.toFixed(0), W - 134, y - 3);
        c2.globalAlpha = 1;
    });
    // matched chime markers — same as spectrogram (green = detected, red dashed = rejected/timeout)
    if (simResult.events) {
        for (const ev of simResult.events) {
            const t = ev.ms / 1000;
            if (t < t0 || t > t1) continue;
            const x = X(t);
            if (ev.type === "detected") {
                c2.strokeStyle = "#3fd078"; c2.lineWidth = 2; c2.setLineDash([]);
                c2.beginPath(); c2.moveTo(x, padT); c2.lineTo(x, H - padB); c2.stroke();
                c2.lineWidth = 1;
                c2.fillStyle = "#3fd078";
                c2.font = "11px ui-monospace,monospace";
                c2.fillText("✓ " + (ev.chimeName || "?"), x + 3, padT + 12);
            } else if (/discount|timeout/.test(ev.type)) {
                c2.strokeStyle = "rgba(255,92,92,0.7)"; c2.setLineDash([4, 3]); c2.lineWidth = 1;
                c2.beginPath(); c2.moveTo(x, padT); c2.lineTo(x, H - padB); c2.stroke();
                c2.setLineDash([]);
            }
        }
    }
    // selected tick marker
    const sel = ticks[simTickIdx];
    if (sel) {
        const x = X(sel.ms / 1000);
        c2.strokeStyle = "#e8edf2"; c2.lineWidth = 1;
        c2.beginPath(); c2.moveTo(x, padT); c2.lineTo(x, H - padB); c2.stroke();
    }
    // bin legend
    for (let b = 0; b < nf; b++) {
        c2.fillStyle = SIM_COLORS[b % SIM_COLORS.length];
        c2.fillText(simResult.meta.freqs[b].toFixed(1) + "Hz", 6, padT + 14 + (b % 4) * 11);
    }
}

/* ---------- events, diagnostics, navigation ---------- */

function evLabel(type) {
    switch (type) {
        case "detected": return "DETECTED";
        case "pattern_start": return "pattern start";
        case "chord_match": return "step match";
        case "falling_edge": return "falling edge";
        case "min_duration_discount": return "rejected · min duration";
        case "max_duration_timeout": return "timeout · max duration";
        case "step_timeout": return "timeout · step window";
        case "released": return "released (re-armed)";
        default: return type;
    }
}

function evDetail(ev) {
    switch (ev.type) {
        case "pattern_start": return `step 1/${ev.numSteps} · peak ${ev.peakDb.toFixed(1)} dB`;
        case "chord_match": return `step ${ev.step + 1}/${ev.numSteps} · peak ${ev.peakDb.toFixed(1)} dB`;
        case "falling_edge": return `step ${ev.step + 1} stopped`;
        case "detected": return "pattern complete";
        case "min_duration_discount": return `< min ${ev.min} ms`;
        case "max_duration_timeout": return `> max ${ev.max} ms (matched ${ev.matched}/${ev.numSteps})`;
        case "step_timeout": return `step ${ev.step + 1} window ended at ${ev.windowEnd} ms`;
        case "released": return `sensor OFF · held ${ev.holdMs} ms`;
        default: return "";
    }
}

function renderSimEvents() {
    const host = $("simEvents");
    const evs = simResult.events;
    if (!evs.length) { host.innerHTML = '<div class="empty">no events — nothing matched</div>'; return; }
    const attemptStart = {}; // chime name -> ms of the current pattern attempt start
    let html = "";
    for (const ev of evs) {
        const name = ev.chimeName || "?";
        let rel;
        if (ev.type === "pattern_start") { attemptStart[name] = ev.ms; rel = 0; }
        else rel = Math.max(0, ev.ms - (attemptStart[name] ?? ev.ms));
        const cls = ev.type === "detected" ? "det" : /discount|timeout/.test(ev.type) ? "rej" : "inf";
        const abs = (ev.ms / 1000).toFixed(3);
        html += `<div class="ev ${cls}" data-ms="${ev.ms}" title="file time ${abs} s"><b>+${rel} ms</b><span class="evc">${name}</span><span class="evt">${evLabel(ev.type)}</span><span class="evd">${evDetail(ev)}</span><span class="evts">${abs} s</span></div>`;
    }
    host.innerHTML = html;
    host.querySelectorAll(".ev").forEach(el => el.onclick = () => jumpTo(+el.dataset.ms / 1000));
}

function renderSimDiag() {
    const host = $("simDiag");
    const ticks = simResult.ticks;
    const tk = ticks[simTickIdx];
    if (!tk) { host.innerHTML = ""; $("simDiagTick").textContent = "–"; return; }
    $("simDiagTick").textContent = `#${tk.tick} / ${ticks.length - 1} · t = ${(tk.ms / 1000).toFixed(3)} s · tick span ${tk.durationMs} ms`;
    let html = `<table class="diag"><thead><tr>
        <th>chime</th><th>bin</th><th>spec</th><th>floor</th><th>eff thr</th><th>thr</th><th>local bg</th><th>prom</th><th>prom gate</th><th>onset Δ</th><th>onset gate</th><th>state</th>
    </tr></thead><tbody>`;
    tk.diag.forEach((d, di) => {
        const snap = tk.chimes[di];
        const stateTxt = d.active
            ? `active · step ${Math.min(d.step + 1, d.numSteps)}/${d.numSteps}${snap.needFallingEdge ? " · awaiting fall" : ""} · +${snap.elapsedMs} ms`
            : (snap.latched ? "latched" : "idle");
        if (!d.bins.length) {
            html += `<tr><td>${d.name}</td><td colspan="11">${stateTxt}</td></tr>`;
            return;
        }
        d.bins.forEach((b, bi) => {
            const floor = tk.bins[b.bin] ? tk.bins[b.bin].floor : null;
            html += `<tr class="${b.thresholdOk && b.prominenceOk && b.onsetOk ? "dg-ok" : "dg-bad"}">
                <td>${bi === 0 ? d.name : ""}</td>
                <td>${b.freq.toFixed(1)} Hz</td>
                <td>${b.db.toFixed(1)}</td>
                <td>${floor == null ? "—" : floor.toFixed(1)}</td>
                <td>${b.effThreshold.toFixed(1)}</td>
                <td class="g ${b.thresholdOk ? "ok" : "bad"}">${b.thresholdOk ? "PASS" : "FAIL"}</td>
                <td>${b.localBg.toFixed(1)}</td>
                <td>${b.prominence.toFixed(1)}</td>
                <td class="g ${b.prominenceOk ? "ok" : "bad"}">${b.prominenceOk ? "PASS" : "FAIL"}</td>
                <td>${b.onset.toFixed(1)}</td>
                <td class="g ${b.onsetOk ? "ok" : "bad"}">${b.onsetOk ? "PASS" : "FAIL"}</td>
                <td class="meta">${bi === 0 ? stateTxt : ""}</td>
            </tr>`;
        });
    });
    html += "</tbody></table>";
    host.innerHTML = html;
}

function jumpTo(t) {
    if (!stft) return;
    const dur = viewEnd - viewStart;
    viewStart = t - dur / 2;
    viewEnd = t + dur / 2;
    clampView();
    if (simResult && simResult.ticks.length) {
        let best = 0, bd = Infinity;
        simResult.ticks.forEach((tk, i) => {
            const d = Math.abs(tk.ms / 1000 - t);
            if (d < bd) { bd = d; best = i; }
        });
        simTickIdx = best;
    }
    renderSimChart();
    renderSimDiag();
    render();
}

function simStripClick(e) {
    if (!simResult) return;
    const cv = $("simStrip");
    const rect = cv.getBoundingClientRect();
    const mx = (e.clientX - rect.left) * (cv.width / rect.width);
    const dur = simResult.meta.durationMs / 1000;
    jumpTo(((mx - 96) / (cv.width - 100)) * dur);
}

function simChartClick(e) {
    if (!simResult || !stft) return;
    const cv = $("simChart");
    const rect = cv.getBoundingClientRect();
    const mx = (e.clientX - rect.left) * (cv.width / rect.width);
    const W = cv.width;
    const t0 = viewStart, t1 = viewEnd;
    const t = t0 + mx / W * (t1 - t0);
    let best = -1, bd = Infinity;
    simResult.ticks.forEach((tk, i) => {
        const d = Math.abs(tk.ms / 1000 - t);
        if (d < bd) { bd = d; best = i; }
    });
    if (best >= 0) { simTickIdx = best; renderSimChart(); renderSimDiag(); }
}

/* ---------- import from segments / test signal / YAML export ---------- */

function importFromSegments() {
    const sel = segments.filter(s => s.selected);
    if (!sel.length) { setMsg("Select segments first (click them on the spectrogram or tick select-all)."); return; }
    if (!simChimeList.length) addSimChime();
    const c = simChimeList[0];
    const anchor = sel[0].start;
    c.steps = sel.map((sg, i) => {
        const freqs = sg.peaks.filter(p => p.selected).map(p => p.f);
        return {
            chord: (freqs.length ? freqs : [0]).join(", "),
            time: i === 0 ? "" : (sg.start - anchor).toFixed(2)
        };
    });
    c.enabled = true;
    buildSimChimes();
    scheduleSimRun();
    setMsg(`Imported ${sel.length} selected segment(s) into "${c.name}".`);
    setTimeout(() => setMsg(""), 4000);
}

function genTestSignal() {
    if (micCapturing) { setMsg("Stop the mic capture first."); return; }
    let c = simChimeList.find(x => x.enabled);
    if (!c) {
        if (!simChimeList.length) addSimChime();
        c = simChimeList[0];
        c.enabled = true;
    }
    const steps = c.steps.map(s => parseFreqs(s.chord));
    if (!steps.length || steps.some(f => !f.length)) { setMsg("That chime needs at least one valid chord frequency."); return; }
    const fs = TARGET_FS;
    const stepDur = 0.25, gap = 0.15, lead = 0.6, tail = 0.8;
    const total = lead + steps.length * stepDur + (steps.length - 1) * gap + tail;
    const len = Math.round(total * fs);
    const data = new Float32Array(len);
    for (let i = 0; i < len; i++) data[i] = (Math.random() * 2 - 1) * 0.001;
    steps.forEach((chord, si) => {
        const t0 = lead + si * (stepDur + gap);
        const i0 = Math.round(t0 * fs), i1 = Math.min(len, Math.round((t0 + stepDur) * fs));
        for (const f of chord) {
            const w = 2 * Math.PI * f / fs;
            for (let i = i0; i < i1; i++) data[i] += 0.4 * Math.sin(w * i);
        }
    });
    const ctx = newAudioCtx();
    const buf = ctx.createBuffer(1, len, fs);
    buf.copyToChannel(data, 0);
    // align pattern timestamps with the synthesized bursts and relax the
    // limits so the short test tones can complete a full pattern
    steps.forEach((chord, si) => { c.steps[si].time = si === 0 ? "" : (si * (stepDur + gap)).toFixed(2); });
    c.minMs = 0;
    if (steps.length === 1) c.prom = 0; // a single bin has no local background to compare against
    fileBuf = buf;
    fname = "test signal (generated)";
    stopPlayback();
    buildSimChimes();
    setMsg(`Synthesized ${steps.length}-step test signal (${total.toFixed(1)} s) — analyzing…`);
    setTimeout(runPipeline, 0);
}

function download(name, text, mime) {
    const blob = new Blob([text], { type: mime || "text/plain" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = name;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 500);
}

function buildSimYamlText() {
    const shared = simSharedCfg();
    const valid = simChimeList.filter(c => c.enabled && c.steps.length && c.steps.every(s => parseFreqs(s.chord).length));
    if (!valid.length) return "";
    const L = [];
    L.push("# ESPHome `chime` component — exported from Chime Detector Simulator");
    L.push("# Each entry under `chimes:` is a binary sensor. Replace `mic1` with your microphone source id.");
    L.push("chime:");
    L.push("  microphone: mic1");
    L.push("  passive: true");
    L.push(`  window_size: ${shared.windowSize}`);
    L.push(`  tick_interval: ${shared.tickIntervalMs}ms`);
    L.push(`  guard_separation_hz: ${shared.guardSeparationHz}`);
    L.push(`  noise_floor_alpha_down: ${shared.noiseFloorAlphaDown}`);
    L.push(`  noise_floor_alpha_up: ${shared.noiseFloorAlphaUp}`);
    L.push("  chimes:");
    valid.forEach(c => {
        L.push(`    - name: "${(c.name || "chime").replace(/"/g, "'")}"`);
        L.push("      pattern:");
        c.steps.forEach(s => {
            const freqs = parseFreqs(s.chord).map(f => f + "Hz");
            L.push(`        - chord: [${freqs.join(", ")}]`);
            const t = (s.time || "").trim();
            if (t) {
                const v = parseFloat(t);
                if (isFinite(v) && v >= 0) L.push(`          time: ${v}`);
            }
        });
        L.push("      duration:");
        L.push(`        min: ${Math.round(simNum(c.minMs, 0))}ms`);
        L.push(`        max: ${Math.round(simNum(c.maxMs, 8000))}ms`);
        L.push(`      threshold: ${simNum(c.thr, -50).toFixed(1)}`);
        L.push(`      snr_margin_db: ${simNum(c.snr, 8).toFixed(1)}`);
        L.push(`      prominence_db: ${simNum(c.prom, 0).toFixed(1)}`);
        L.push(`      onset_contrast_db: ${simNum(c.onset, 8).toFixed(1)}`);
        L.push(`      tail_grace: ${Math.round(simNum(c.tail, 2000))}ms`);
    });
    return L.join("\n") + "\n";
}

function renderSimYaml() {
    $("yaml").textContent = buildSimYamlText() || "# (no enabled chime with valid chords yet)";
}

function exportSimYaml() {
    const text = buildSimYamlText();
    if (!text) { setMsg("Nothing to export — add an enabled chime with valid chords."); return; }
    const singleStep = simChimeList.some(c => c.enabled && c.steps.length < 2 && c.steps.every(s => parseFreqs(s.chord).length));
    download("chime_config.yaml", text, "text/yaml");
    setMsg(singleStep ? "Note: ESPHome requires at least 2 pattern steps per chime — single-step chimes will fail validation." : "YAML downloaded.");
    setTimeout(() => setMsg(""), 5000);
}

/* ---------- chime sim wiring ---------- */

$("simWin").value = SIM_DEF.windowSize;
$("simTick").value = SIM_DEF.tickIntervalMs;
$("simGuard").value = SIM_DEF.guardHz;
$("simAd").value = SIM_DEF.alphaDown;
$("simAu").value = SIM_DEF.alphaUp;
["simWin", "simTick", "simGuard", "simAd", "simAu"].forEach(id => $(id).addEventListener("change", () => { renderSimYaml(); scheduleSimRun(); }));
$("btnSimAdd").onclick = () => { addSimChime(); buildSimChimes(); };
$("btnSimImport").onclick = importFromSegments;
$("btnSimTest").onclick = genTestSignal;
$("btnSimRun").onclick = () => { if (!stft) { setMsg("Load an audio file first (or use 'generate test signal')."); return; } runSim(); };
$("btnSimYaml").onclick = exportSimYaml;
$("simStrip").addEventListener("click", simStripClick);
$("simChart").addEventListener("click", simChartClick);
$("diagPrev").onclick = () => { if (!simResult) return; simTickIdx = Math.max(0, simTickIdx - 1); renderSimChart(); renderSimDiag(); };
$("diagNext").onclick = () => { if (!simResult) return; simTickIdx = Math.min(simResult.ticks.length - 1, simTickIdx + 1); renderSimChart(); renderSimDiag(); };
addSimChime();
buildSimChimes();
