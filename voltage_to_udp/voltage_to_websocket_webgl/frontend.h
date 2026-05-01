const char* html_index = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>PRO AUTO SCOPE</title>
  <style>
    body { background: #000; color: #0f0; font-family: 'Consolas', 'Courier New', monospace; margin: 0; overflow: hidden; }
    #ui { position: absolute; top: 10px; left: 10px; z-index: 10; background: rgba(20,20,20,0.8); padding: 15px; border: 1px solid #444; border-radius: 4px; box-shadow: 0 0 10px rgba(0,255,0,0.2); pointer-events: auto; }
    canvas { width: 100vw; height: 100vh; display: block; position: absolute; top: 0; left: 0; }
    .chan { margin-bottom: 8px; font-size: 14px; display: flex; align-items: center; }
    .chan-color { width: 12px; height: 12px; margin-right: 8px; border-radius: 2px; }
    .c0 { background: #ffff00; color: #ffff00; }
    .c1 { background: #00ffff; color: #00ffff; }
    .c2 { background: #ff00ff; color: #ff00ff; }
    .c3 { background: #00ff00; color: #00ff00; }
    .btn { background: #333; color: #eee; border: 1px solid #666; padding: 6px 12px; cursor: pointer; margin-right: 5px; font-family: inherit; transition: all 0.2s; }
    .btn:hover { background: #555; border-color: #999; }
    .btn.active { background: #006600; border-color: #00ff00; color: #fff; }
    .stat-overlay { position: absolute; bottom: 10px; right: 15px; color: #008800; font-size: 12px; text-shadow: 1px 1px #000; pointer-events: none; }
    .trigger-indicator { display: inline-block; width: 10px; height: 10px; border-radius: 50%; background: #333; margin-left: 5px; }
    .trigger-active { background: #f00; box-shadow: 0 0 5px #f00; }
    h2 { margin: 0 0 10px 0; font-size: 16px; letter-spacing: 1px; color: #aaa; border-bottom: 1px solid #333; padding-bottom: 5px; }
    .wifi-config { margin-top: 15px; border-top: 1px solid #333; padding-top: 10px; }
    .wifi-config input { background: #111; color: #0f0; border: 1px solid #444; padding: 4px; margin-bottom: 5px; width: 150px; font-family: inherit; }
  </style>
</head>
<body>
  <canvas id="canvas"></canvas>
  <div id="ui">
    <h2>ENGINE ANALYZER</h2>
    <div class="chan"><div class="chan-color c0"></div>CH1: <span id="v0">0.00</span>V <div id="t0" class="trigger-indicator"></div></div>
    <div class="chan"><div class="chan-color c1"></div>CH2: <span id="v1">0.00</span>V <div id="t1" class="trigger-indicator"></div></div>
    <div class="chan"><div class="chan-color c2"></div>CH3: <span id="v2">0.00</span>V <div id="t2" class="trigger-indicator"></div></div>
    <div class="chan"><div class="chan-color c3"></div>CH4: <span id="v3">0.00</span>V <div id="t3" class="trigger-indicator"></div></div>
    <hr style="border: 0; border-top: 1px solid #333; margin: 10px 0;">
    <div>
      <div style="margin-bottom: 5px; font-size: 12px; color: #888;">TRIGGER MODE</div>
      <button id="btnMode0" class="btn active" onclick="setMode(0)">INDEPENDENT</button>
      <button id="btnMode1" class="btn" onclick="setMode(1)">CAM WHEEL</button>
    </div>
    <div class="wifi-config">
      <div style="margin-bottom: 5px; font-size: 12px; color: #888;">WIFI CONFIG (STA)</div>
      <input type="text" id="ssid" placeholder="SSID"><br>
      <input type="password" id="pass" placeholder="PASSWORD"><br>
      <button class="btn" onclick="saveWifi()">SAVE & REBOOT</button>
    </div>
  </div>
  <div class="stat-overlay" id="stats">LINK: DOWN | FPS: 0 | SEQ: 0</div>

  <script id="vs" type="x-shader/x-vertex">
    attribute float a_y;
    attribute float a_x;
    uniform float u_y_offset;
    uniform float u_y_scale;
    void main() {
      gl_Position = vec4(a_x, (a_y / 4095.0 * 2.0 - 1.0) * u_y_scale + u_y_offset, 0.0, 1.0);
    }
  </script>
  <script id="fs" type="x-shader/x-fragment">
    precision mediump float;
    uniform vec4 u_color;
    void main() {
      gl_FragColor = u_color;
    }
  </script>

  <script>
    const canvas = document.getElementById('canvas');
    const gl = canvas.getContext('webgl', { antialias: true });

    function createShader(gl, type, source) {
      const s = gl.createShader(type);
      gl.shaderSource(s, source);
      gl.compileShader(s);
      return s;
    }

    const program = gl.createProgram();
    gl.attachShader(program, createShader(gl, gl.VERTEX_SHADER, document.getElementById('vs').text));
    gl.attachShader(program, createShader(gl, gl.FRAGMENT_SHADER, document.getElementById('fs').text));
    gl.linkProgram(program);
    gl.useProgram(program);

    const SAMPLES = 512;
    const CHANNELS = 4;
    const PHOSPHOR_COUNT = 8;

    let history = [];
    const xCoords = new Float32Array(SAMPLES);
    for(let i=0; i<SAMPLES; i++) xCoords[i] = (i / (SAMPLES-1)) * 2.0 - 1.0;

    const xBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, xBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, xCoords, gl.STATIC_DRAW);

    const yBuffers = Array.from({length: CHANNELS * PHOSPHOR_COUNT}, () => gl.createBuffer());

    const colors = [
      [1, 1, 0, 1], // Yellow
      [0, 1, 1, 1], // Cyan
      [1, 0, 1, 1], // Magenta
      [0, 1, 0, 1]  // Green
    ];

    function render() {
      gl.viewport(0, 0, canvas.width, canvas.height);
      gl.clearColor(0.01, 0.01, 0.01, 1);
      gl.clear(gl.COLOR_BUFFER_BIT);

      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);

      if (history.length === 0) return;

      const a_x = gl.getAttribLocation(program, 'a_x');
      const a_y = gl.getAttribLocation(program, 'a_y');
      const u_color = gl.getUniformLocation(program, 'u_color');
      const u_y_offset = gl.getUniformLocation(program, 'u_y_offset');
      const u_y_scale = gl.getUniformLocation(program, 'u_y_scale');

      for (let h = 0; h < history.length; h++) {
        const entry = history[h];
        const alpha = (h + 1) / history.length;

        for (let c = 0; c < CHANNELS; c++) {
          const bufIdx = (h * CHANNELS) + c;
          gl.bindBuffer(gl.ARRAY_BUFFER, yBuffers[bufIdx]);

          const chanData = new Float32Array(SAMPLES);
          for(let i=0; i<SAMPLES; i++) chanData[i] = entry.data[i * CHANNELS + c];
          gl.bufferData(gl.ARRAY_BUFFER, chanData, gl.STREAM_DRAW);

          gl.enableVertexAttribArray(a_x);
          gl.bindBuffer(gl.ARRAY_BUFFER, xBuffer);
          gl.vertexAttribPointer(a_x, 1, gl.FLOAT, false, 0, 0);

          gl.enableVertexAttribArray(a_y);
          gl.bindBuffer(gl.ARRAY_BUFFER, yBuffers[bufIdx]);
          gl.vertexAttribPointer(a_y, 1, gl.FLOAT, false, 0, 0);

          const color = [...colors[c]];
          color[3] = alpha * 0.8;
          gl.uniform4fv(u_color, color);
          gl.uniform1f(u_y_offset, 0.6 - c * 0.4);
          gl.uniform1f(u_y_scale, 0.25);
          gl.drawArrays(gl.LINE_STRIP, 0, SAMPLES);

          if (h === history.length - 1) {
            document.getElementById('v'+c).innerText = (entry.data[c] * 3.3 / 4095).toFixed(2);
            const tInd = document.getElementById('t'+c);
            if (entry.trig & (1 << c)) {
              tInd.classList.add('trigger-active');
              setTimeout(() => tInd.classList.remove('trigger-active'), 100);
            }
          }
        }
      }
    }

    let pktCount = 0;
    let lastTime = performance.now();
    let fps = 0;

    function connect() {
      const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
      const ws = new WebSocket(protocol + '//' + location.host + '/ws');
      ws.binaryType = 'arraybuffer';
      ws.onopen = () => { document.getElementById('stats').innerText = 'LINK: UP'; };
      ws.onclose = () => {
        document.getElementById('stats').innerText = 'LINK: DOWN';
        setTimeout(connect, 2000);
      };
      ws.onmessage = (e) => {
        if (typeof e.data === 'string') return;
        const view = new DataView(e.data);
        const seq = view.getUint32(0, true);
        const trig = view.getUint8(8);
        const data = new Uint16Array(e.data, 9, SAMPLES * CHANNELS);

        history.push({seq, trig, data});
        if(history.length > PHOSPHOR_COUNT) history.shift();

        pktCount++;
        const now = performance.now();
        if(now - lastTime > 1000) {
          fps = pktCount;
          pktCount = 0;
          lastTime = now;
        }
        document.getElementById('stats').innerText = `LINK: UP | FPS: ${fps} | SEQ: ${seq}`;
        requestAnimationFrame(render);
      };
      window.ws = ws;
    }

    function setMode(m) {
      if(window.ws && window.ws.readyState === WebSocket.OPEN) {
        window.ws.send('M' + m);
        document.getElementById('btnMode0').classList.toggle('active', m === 0);
        document.getElementById('btnMode1').classList.toggle('active', m === 1);
      }
    }

    function saveWifi() {
      const s = document.getElementById('ssid').value;
      const p = document.getElementById('pass').value;
      if(window.ws && window.ws.readyState === WebSocket.OPEN) {
        window.ws.send('W;' + s + ';' + p);
      }
    }

    window.onresize = () => {
      canvas.width = window.innerWidth;
      canvas.height = window.innerHeight;
      gl.viewport(0, 0, canvas.width, canvas.height);
    };
    window.onresize();
    connect();
  </script>
</body>
</html>
)rawliteral";
