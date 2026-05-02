const char* html_index = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>PRO AUTO SCOPE</title>
  <style>
    body { background: #000; color: #0f0; font-family: 'Consolas', 'Courier New', monospace; margin: 0; overflow: hidden; }
    #ui { position: absolute; top: 10px; left: 10px; z-index: 10; background: rgba(20,20,20,0.8); padding: 15px; border: 1px solid #444; border-radius: 4px; box-shadow: 0 0 10px rgba(0,255,0,0.2); pointer-events: auto; max-height: 90vh; overflow-y: auto; }
    canvas { width: 100vw; height: 100vh; display: block; position: absolute; top: 0; left: 0; }
    .chan { margin-bottom: 8px; font-size: 14px; }
    .chan-row { display: flex; align-items: center; margin-bottom: 5px; }
    .chan-color { width: 12px; height: 12px; margin-right: 8px; border-radius: 2px; }
    .c0 { background: #ffff00; } .c1 { background: #00ffff; } .c2 { background: #ff00ff; } .c3 { background: #00ff00; }
    .btn { background: #333; color: #eee; border: 1px solid #666; padding: 4px 8px; cursor: pointer; margin-right: 5px; font-family: inherit; font-size: 11px; }
    .btn:hover { background: #555; }
    .btn.active { background: #006600; border-color: #00ff00; }
    .stat-overlay { position: absolute; bottom: 10px; right: 15px; color: #008800; font-size: 12px; text-shadow: 1px 1px #000; pointer-events: none; }
    .trigger-indicator { display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: #333; margin-left: 5px; }
    .trigger-active { background: #f00; box-shadow: 0 0 5px #f00; }
    h2 { margin: 0 0 10px 0; font-size: 14px; color: #aaa; border-bottom: 1px solid #333; padding-bottom: 5px; }
    .ctrl-group { margin-top: 10px; border-top: 1px solid #333; padding-top: 5px; }
    label { font-size: 10px; color: #888; display: block; margin-bottom: 2px; }
    input[type=range] { width: 100%; height: 4px; margin-bottom: 8px; accent-color: #0f0; }
    .val-display { float: right; color: #0f0; }
  </style>
</head>
<body>
  <canvas id="canvas"></canvas>
  <div id="ui">
    <h2>ENGINE ANALYZER</h2>
    <div id="chan-controls"></div>

    <div class="ctrl-group">
      <label>TIMEBASE (Horizontal Zoom) <span class="val-display" id="val-tb">1.0</span></label>
      <input type="range" id="ctrl-tb" min="0.1" max="5.0" step="0.1" value="1.0" oninput="updateControls()">
    </div>

    <div class="ctrl-group">
      <div style="margin-bottom: 5px; font-size: 10px; color: #888;">TRIGGER MODE</div>
      <button id="btnMode0" class="btn active" onclick="setMode(0)">INDEPENDENT</button>
      <button id="btnMode1" class="btn" onclick="setMode(1)">CAM WHEEL</button>
    </div>

    <div class="ctrl-group">
      <div style="margin-bottom: 5px; font-size: 10px; color: #888;">DEBUG / SIMULATION</div>
      <button id="btnSim" class="btn" onclick="toggleSim()">SIMULATION: OFF</button>
    </div>

    <div class="ctrl-group">
      <div style="margin-bottom: 5px; font-size: 10px; color: #888;">WIFI CONFIG</div>
      <input style="background:#111;color:#0f0;border:1px solid #444;width:80px;font-size:10px;" type="text" id="ssid" placeholder="SSID">
      <input style="background:#111;color:#0f0;border:1px solid #444;width:80px;font-size:10px;" type="password" id="pass" placeholder="PASS">
      <button class="btn" onclick="saveWifi()">REBOOT</button>
    </div>
  </div>
  <div class="stat-overlay" id="stats">LINK: DOWN | FPS: 0</div>

  <script id="vs" type="x-shader/x-vertex">
    attribute float a_y;
    attribute float a_x;
    uniform float u_offset;
    uniform float u_scale;
    uniform float u_tb;
    void main() {
      gl_Position = vec4(a_x * u_tb, (a_y / 4095.0 * 2.0 - 1.0) * u_scale + u_offset, 0.0, 1.0);
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
    let ws;
    let simOn = false;

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

    const colors = [ [1,1,0,1], [0,1,1,1], [1,0,1,1], [0,1,0,1] ];
    let chanSettings = [
      { offset: 0.6, scale: 0.25 },
      { offset: 0.2, scale: 0.25 },
      { offset: -0.2, scale: 0.25 },
      { offset: -0.6, scale: 0.25 }
    ];

    function initUI() {
      const container = document.getElementById('chan-controls');
      container.innerHTML = '';
      for(let i=0; i<CHANNELS; i++) {
        const div = document.createElement('div');
        div.className = 'chan';
        div.innerHTML = `
          <div class="chan-row">
            <div class="chan-color c${i}"></div>
            <span>CH${i+1}: <span id="v${i}">0.00</span>V</span>
            <div id="t${i}" class="trigger-indicator"></div>
          </div>
          <label>SCALE <span class="val-display" id="val-s${i}">0.25</span></label>
          <input type="range" id="ctrl-s${i}" min="0.05" max="1.0" step="0.05" value="0.25" oninput="updateControls()">
          <label>OFFSET <span class="val-display" id="val-o${i}">${chanSettings[i].offset}</span></label>
          <input type="range" id="ctrl-o${i}" min="-1.0" max="1.0" step="0.05" value="${chanSettings[i].offset}" oninput="updateControls()">
        `;
        container.appendChild(div);
      }
    }

    function updateControls() {
      for(let i=0; i<CHANNELS; i++) {
        chanSettings[i].scale = parseFloat(document.getElementById('ctrl-s'+i).value);
        chanSettings[i].offset = parseFloat(document.getElementById('ctrl-o'+i).value);
        document.getElementById('val-s'+i).innerText = chanSettings[i].scale.toFixed(2);
        document.getElementById('val-o'+i).innerText = chanSettings[i].offset.toFixed(2);
      }
      document.getElementById('val-tb').innerText = document.getElementById('ctrl-tb').value;
    }

    const gridBuffer = gl.createBuffer();
    function createGrid() {
      const gridPoints = [];
      for(let i=-10; i<=10; i++) {
        gridPoints.push(-1, (i/10 + 1)*2048, 1, (i/10 + 1)*2048);
        gridPoints.push(i/10, 0, i/10, 4095);
      }
      gl.bindBuffer(gl.ARRAY_BUFFER, gridBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(gridPoints), gl.STATIC_DRAW);
    }

    function drawGrid() {
      gl.bindBuffer(gl.ARRAY_BUFFER, gridBuffer);
      const a_x = gl.getAttribLocation(program, 'a_x');
      const a_y = gl.getAttribLocation(program, 'a_y');
      gl.enableVertexAttribArray(a_x);
      gl.vertexAttribPointer(a_x, 1, gl.FLOAT, false, 8, 0);
      gl.enableVertexAttribArray(a_y);
      gl.vertexAttribPointer(a_y, 1, gl.FLOAT, false, 8, 4);

      gl.uniform4f(gl.getUniformLocation(program, 'u_color'), 0.1, 0.1, 0.1, 1);
      gl.uniform1f(gl.getUniformLocation(program, 'u_offset'), 0);
      gl.uniform1f(gl.getUniformLocation(program, 'u_scale'), 1);
      gl.uniform1f(gl.getUniformLocation(program, 'u_tb'), 1);
      gl.drawArrays(gl.LINES, 0, 44);
    }

    function render() {
      gl.viewport(0, 0, canvas.width, canvas.height);
      gl.clearColor(0.01, 0.01, 0.01, 1);
      gl.clear(gl.COLOR_BUFFER_BIT);
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE);

      drawGrid();

      if (history.length === 0) return;

      const a_x = gl.getAttribLocation(program, 'a_x');
      const a_y = gl.getAttribLocation(program, 'a_y');
      const u_color = gl.getUniformLocation(program, 'u_color');
      const u_offset = gl.getUniformLocation(program, 'u_offset');
      const u_scale = gl.getUniformLocation(program, 'u_scale');
      const u_tb = gl.getUniformLocation(program, 'u_tb');

      const tbVal = parseFloat(document.getElementById('ctrl-tb').value);

      for (let h = 0; h < history.length; h++) {
        const entry = history[h];
        const alpha = (h + 1) / history.length;
        for (let c = 0; c < CHANNELS; c++) {
          const bufIdx = (h * CHANNELS) + c;
          gl.bindBuffer(gl.ARRAY_BUFFER, yBuffers[bufIdx]);
          if(h === history.length-1) {
             const chanData = new Float32Array(SAMPLES);
             for(let i=0; i<SAMPLES; i++) chanData[i] = entry.data[i * CHANNELS + c];
             gl.bufferData(gl.ARRAY_BUFFER, chanData, gl.STREAM_DRAW);
          }
          gl.enableVertexAttribArray(a_x);
          gl.bindBuffer(gl.ARRAY_BUFFER, xBuffer);
          gl.vertexAttribPointer(a_x, 1, gl.FLOAT, false, 0, 0);
          gl.enableVertexAttribArray(a_y);
          gl.bindBuffer(gl.ARRAY_BUFFER, yBuffers[bufIdx]);
          gl.vertexAttribPointer(a_y, 1, gl.FLOAT, false, 0, 0);

          const color = [...colors[c]];
          color[3] = alpha * 0.6;
          gl.uniform4fv(u_color, color);
          gl.uniform1f(u_offset, chanSettings[c].offset);
          gl.uniform1f(u_scale, chanSettings[c].scale);
          gl.uniform1f(u_tb, tbVal);
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

    let pktCount = 0, lastTime = performance.now(), fps = 0;

    function connect() {
      const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
      ws = new WebSocket(protocol + '//' + location.host + '/ws');
      ws.binaryType = 'arraybuffer';
      ws.onopen = () => { document.getElementById('stats').innerText = 'LINK: UP'; };
      ws.onclose = () => { document.getElementById('stats').innerText = 'LINK: DOWN'; setTimeout(connect, 2000); };
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
        if(now - lastTime > 1000) { fps = pktCount; pktCount = 0; lastTime = now; }
        document.getElementById('stats').innerText = `LINK: UP | FPS: ${fps} | SEQ: ${seq}`;
        requestAnimationFrame(render);
      };
    }

    function setMode(m) {
      if(ws && ws.readyState === WebSocket.OPEN) {
        ws.send('M' + m);
        document.getElementById('btnMode0').classList.toggle('active', m === 0);
        document.getElementById('btnMode1').classList.toggle('active', m === 1);
      }
    }

    function toggleSim() {
      simOn = !simOn;
      if(ws && ws.readyState === WebSocket.OPEN) {
        ws.send('S' + (simOn ? '1' : '0'));
        document.getElementById('btnSim').innerText = 'SIMULATION: ' + (simOn ? 'ON' : 'OFF');
        document.getElementById('btnSim').classList.toggle('active', simOn);
      }
    }

    function saveWifi() {
      const s = document.getElementById('ssid').value, p = document.getElementById('pass').value;
      if(ws && ws.readyState === WebSocket.OPEN) ws.send('W;' + s + ';' + p);
    }

    window.onresize = () => { canvas.width = window.innerWidth; canvas.height = window.innerHeight; gl.viewport(0, 0, canvas.width, canvas.height); };
    window.onresize();
    initUI();
    createGrid();
    connect();
  </script>
</body>
</html>
)rawliteral";
