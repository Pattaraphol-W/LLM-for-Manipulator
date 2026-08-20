package main

import (
	"encoding/json"
	"fmt"
	htmltemplate "html/template"
	"net/http"
	"os"
	"sync"
	"time"
)

// liveState is what the web UI shows: the exact frame handed to the model, plus
// the description as it is generated. Guarded because the inference loop writes
// it while HTTP handlers read it.
type liveState struct {
	mu          sync.RWMutex
	frame       []byte
	description string
	status      string
	luma        float64
	seconds     float64
	frameNum    int
	startedAt   time.Time
}

var live = &liveState{status: "starting"}

func (s *liveState) setStatus(status string) {
	s.mu.Lock()
	s.status = status
	s.mu.Unlock()
}

// newFrame publishes the JPEG that is about to be sent to the model.
func (s *liveState) newFrame(path string, luma float64) {
	data, err := os.ReadFile(path)
	s.mu.Lock()
	defer s.mu.Unlock()
	if err == nil {
		s.frame = data
	}
	s.luma = luma
	s.frameNum++
	s.description = ""
	s.status = "thinking"
	s.startedAt = time.Now()
}

// appendToken streams generated text out to the browser as it appears.
func (s *liveState) appendToken(tok string) {
	s.mu.Lock()
	s.description += tok
	s.mu.Unlock()
}

func (s *liveState) done(seconds float64) {
	s.mu.Lock()
	s.seconds = seconds
	s.status = "done"
	s.mu.Unlock()
}

func (s *liveState) snapshot() map[string]any {
	s.mu.RLock()
	defer s.mu.RUnlock()
	elapsed := 0.0
	if s.status == "thinking" && !s.startedAt.IsZero() {
		elapsed = time.Since(s.startedAt).Seconds()
	}
	return map[string]any{
		"description": s.description,
		"status":      s.status,
		"luma":        s.luma,
		"seconds":     s.seconds,
		"frame":       s.frameNum,
		"elapsed":     elapsed,
	}
}

func serve(addr string) {
	tmpl := htmltemplate.Must(htmltemplate.New("page").Parse(pageHTML))

	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/" {
			http.NotFound(w, r)
			return
		}
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		tmpl.Execute(w, nil)
	})

	http.HandleFunc("/frame.jpg", func(w http.ResponseWriter, r *http.Request) {
		live.mu.RLock()
		data := live.frame
		live.mu.RUnlock()
		if len(data) == 0 {
			http.Error(w, "no frame yet", http.StatusServiceUnavailable)
			return
		}
		w.Header().Set("Content-Type", "image/jpeg")
		w.Header().Set("Cache-Control", "no-store")
		w.Write(data)
	})

	http.HandleFunc("/api", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		w.Header().Set("Cache-Control", "no-store")
		json.NewEncoder(w).Encode(live.snapshot())
	})

	go func() {
		if err := http.ListenAndServe(addr, nil); err != nil {
			fmt.Fprintln(os.Stderr, "web UI failed:", err)
		}
	}()
	fmt.Printf("web UI on http://<board-ip>%s\n", addr)
}

const pageHTML = `<!doctype html>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>vlmcam</title>
<style>
 :root{color-scheme:dark}
 body{margin:0;padding:1.2rem;background:#111;color:#eee;
      font:15px/1.55 ui-sans-serif,system-ui,sans-serif}
 .wrap{max-width:760px;margin:0 auto}
 h1{font-size:1rem;font-weight:600;color:#9aa;margin:0 0 .8rem;letter-spacing:.04em}
 img{width:100%;border-radius:8px;background:#000;display:block}
 .meta{display:flex;gap:1.2rem;flex-wrap:wrap;margin:.7rem 0;color:#8a8f98;font-size:.85rem}
 .meta b{color:#ddd;font-weight:600}
 .desc{background:#1a1a1d;border-radius:8px;padding:1rem;min-height:5rem;white-space:pre-wrap}
 .dot{display:inline-block;width:.5rem;height:.5rem;border-radius:50%;background:#3b3}
 .thinking .dot{background:#fa3;animation:p 1s infinite}
 @keyframes p{50%{opacity:.25}}
 .warn{color:#f96}
</style>
<div class="wrap">
  <h1>VLMCAM — WHAT THE MODEL SEES</h1>
  <img id="f" src="/frame.jpg" alt="current frame">
  <div class="meta">
    <span id="st"><span class="dot"></span> <b>—</b></span>
    <span>frame <b id="n">—</b></span>
    <span>luma <b id="l">—</b></span>
    <span>last <b id="s">—</b></span>
  </div>
  <div class="desc" id="d">waiting for the first frame…</div>
</div>
<script>
let seen = -1;
async function tick(){
  try{
    const r = await fetch('/api'); const j = await r.json();
    if(j.frame !== seen){ seen = j.frame; document.getElementById('f').src = '/frame.jpg?t=' + Date.now(); }
    document.getElementById('n').textContent = j.frame;
    document.getElementById('l').textContent = j.luma.toFixed(1) + (j.luma < 25 ? ' ⚠ dark' : '');
    document.getElementById('l').className = j.luma < 25 ? 'warn' : '';
    document.getElementById('s').textContent = j.seconds ? j.seconds.toFixed(1)+'s' : '—';
    const st = document.getElementById('st');
    st.querySelector('b').textContent = j.status === 'thinking'
      ? 'thinking ' + j.elapsed.toFixed(0) + 's' : j.status;
    st.className = j.status === 'thinking' ? 'thinking' : '';
    document.getElementById('d').textContent = j.description || (j.status==='thinking' ? '…' : '');
  }catch(e){}
}
setInterval(tick, 700); tick();
</script>`
