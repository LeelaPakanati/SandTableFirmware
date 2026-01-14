#pragma once

const char WEB_UI_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Sisyphus Table Control</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: #333;
        }

        .container {
            max-width: 800px;
            margin: 0 auto;
        }

        .card {
            background: white;
            border-radius: 12px;
            padding: 24px;
            margin-bottom: 20px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }

        h1 {
            color: white;
            text-align: center;
            margin-bottom: 30px;
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.2);
        }

        h2 {
            color: #667eea;
            margin-bottom: 16px;
            font-size: 1.5em;
            border-bottom: 2px solid #667eea;
            padding-bottom: 8px;
        }

        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 16px;
            margin-bottom: 16px;
        }

        .status-item {
            background: #f7fafc;
            padding: 12px;
            border-radius: 8px;
            border-left: 4px solid #667eea;
        }

        .status-label {
            font-size: 0.875em;
            color: #718096;
            margin-bottom: 4px;
        }

        .status-value {
            font-size: 1.25em;
            font-weight: 600;
            color: #2d3748;
        }

        .status-badge {
            display: inline-block;
            padding: 4px 12px;
            border-radius: 12px;
            font-size: 0.875em;
            font-weight: 600;
        }

        .status-idle {
            background: #48bb78;
            color: white;
        }

        .status-running {
            background: #4299e1;
            color: white;
        }

        .status-paused {
            background: #ed8936;
            color: white;
        }

        .status-clearing {
            background: #9f7aea;
            color: white;
        }

        .status-stopping {
            background: #ed8936;
            color: white;
        }

        .slider-container {
            margin: 20px 0;
        }

        .slider-label {
            display: flex;
            justify-content: space-between;
            margin-bottom: 8px;
            font-weight: 500;
        }

        input[type="range"] {
            width: 100%;
            height: 8px;
            border-radius: 4px;
            background: #e2e8f0;
            outline: none;
            -webkit-appearance: none;
        }

        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
        }

        input[type="range"]::-moz-range-thumb {
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
            border: none;
        }

        .form-group {
            margin-bottom: 16px;
        }

        label {
            display: block;
            margin-bottom: 8px;
            font-weight: 500;
            color: #2d3748;
        }

        select, input[type="file"] {
            width: 100%;
            padding: 12px;
            border: 2px solid #e2e8f0;
            border-radius: 8px;
            font-size: 1em;
            background: white;
        }

        select:focus, input:focus {
            outline: none;
            border-color: #667eea;
        }

        .button-group {
            display: flex;
            gap: 12px;
            flex-wrap: wrap;
        }

        button {
            flex: 1;
            min-width: 120px;
            padding: 12px 24px;
            border: none;
            border-radius: 8px;
            font-size: 1em;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s;
        }

        .btn-primary {
            background: #667eea;
            color: white;
        }

        .btn-primary:hover {
            background: #5a67d8;
        }

        .btn-secondary {
            background: #ed8936;
            color: white;
        }

        .btn-secondary:hover {
            background: #dd6b20;
        }

        .btn-danger {
            background: #f56565;
            color: white;
        }

        .btn-danger:hover {
            background: #e53e3e;
        }

        .btn-success {
            background: #48bb78;
            color: white;
        }

        .btn-success:hover {
            background: #38a169;
        }

        .file-list {
            max-height: 300px;
            overflow-y: auto;
        }

        .file-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 12px;
            background: #f7fafc;
            border-radius: 8px;
            margin-bottom: 8px;
        }

        .file-info {
            flex: 1;
        }

        .file-name {
            font-weight: 600;
            color: #2d3748;
        }

        .file-size {
            font-size: 0.875em;
            color: #718096;
        }

        .btn-small {
            padding: 6px 12px;
            font-size: 0.875em;
            min-width: auto;
        }

        .upload-area {
            border: 2px dashed #cbd5e0;
            border-radius: 8px;
            padding: 24px;
            text-align: center;
            margin-bottom: 16px;
            cursor: pointer;
            transition: all 0.3s;
        }

        .upload-area:hover {
            border-color: #667eea;
            background: #f7fafc;
        }

        .system-info {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 12px;
            font-size: 0.875em;
        }

        .info-item {
            display: flex;
            justify-content: space-between;
            padding: 8px;
            background: #f7fafc;
            border-radius: 6px;
        }

        @media (max-width: 768px) {
            body {
                padding: 10px;
            }

            h1 {
                font-size: 1.75em;
            }

            .button-group {
                flex-direction: column;
            }

            button {
                min-width: 100%;
            }
        }

        .loading {
            opacity: 0.6;
            pointer-events: none;
        }

        .viewer-canvas {
            width: 100%;
            max-width: 400px;
            height: auto;
            border: 2px solid #e2e8f0;
            border-radius: 8px;
            background: #f7fafc;
        }

        .console-container {
            background: #1a1a2e;
            border-radius: 8px;
            padding: 12px;
            font-family: 'Consolas', 'Monaco', 'Courier New', monospace;
            font-size: 12px;
            line-height: 1.4;
            max-height: 300px;
            overflow-y: auto;
            color: #00ff00;
        }

        .console-line {
            margin: 0;
            padding: 1px 0;
            white-space: pre-wrap;
            word-wrap: break-word;
        }

        .console-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 12px;
        }

        .console-status {
            font-size: 0.875em;
            padding: 4px 8px;
            border-radius: 4px;
        }

        .console-connected {
            background: #48bb78;
            color: white;
        }

        .console-disconnected {
            background: #f56565;
            color: white;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Sisyphus Table Control</h1>

        <div class="card">
            <h2>Status</h2>
            <div class="status-grid">
                <div class="status-item">
                    <div class="status-label">State</div>
                    <div class="status-value">
                        <span id="state-badge" class="status-badge status-idle">IDLE</span>
                    </div>
                </div>
                <div class="status-item">
                    <div class="status-label">Current Pattern</div>
                    <div class="status-value" id="current-pattern">None</div>
                </div>
                <div class="status-item">
                    <div class="status-label">LED Brightness</div>
                    <div class="status-value"><span id="status-brightness">50</span>%</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Uptime</div>
                    <div class="status-value" id="uptime">0s</div>
                </div>
            </div>
         </div>

         <div class="card">
             <h2>Position Viewer</h2>
             <canvas id="position-canvas" class="viewer-canvas" width="400" height="400"></canvas>
             <div style="text-align: center; margin-top: 12px;">
                 <div style="font-size: 0.875em; color: #718096;">Live view of ball position and path</div>
                 <div id="position-coords" style="margin-top: 8px; font-family: monospace; font-size: 0.9em; color: #2d3748;">
                     Position: Loading...
                 </div>
             </div>
         </div>

         <div class="card">
             <h2>LED Brightness</h2>
            <div class="slider-container">
                <div class="slider-label">
                    <span>Brightness</span>
                    <span id="brightness-value">50%</span>
                </div>
                <input type="range" id="brightness-slider" min="0" max="100" value="50">
            </div>
            <div class="button-group" style="margin-top: 12px;">
                <button class="btn-danger" id="btn-led-off">Turn Off</button>
            </div>
        </div>

        <div class="card">
            <h2>Motor Speed</h2>
            <div class="slider-container">
                <div class="slider-label">
                    <span>Speed</span>
                    <span id="speed-value">5</span>
                </div>
                <input type="range" id="speed-slider" min="1" max="10" value="5">
            </div>
        </div>

        <div class="card">
            <h2>Pattern Control</h2>
            <div class="form-group">
                <label for="pattern-select">Select Pattern</label>
                <select id="pattern-select">
                    <option value="">Loading patterns...</option>
                </select>
            </div>
            <div class="form-group">
                <label for="clearing-select">Clearing Pattern</label>
                <select id="clearing-select">
                    <option value="none">None (No Clearing)</option>
                    <option value="random">Random</option>
                    <option value="spiral_outward">Spiral Outward</option>
                    <option value="spiral_inward">Spiral Inward</option>
                    <option value="concentric_circles">Concentric Circles</option>
                    <option value="zigzag_radial">Zigzag Radial</option>
                    <option value="petal_flower">Petal Flower</option>
                </select>
            </div>
            <div class="button-group">
                <button class="btn-primary" id="btn-start">▶ Start</button>
                <button class="btn-secondary" id="btn-pause">⏸ Pause</button>
                <button class="btn-danger" id="btn-stop">⏹ Stop</button>
            </div>
            <div class="button-group" style="margin-top: 12px;">
                <button class="btn-secondary" id="btn-home">🏠 Home Device</button>
            </div>
        </div>

        <div class="card">
            <h2>Playlist</h2>
            <div class="form-group">
                <label for="playlist-mode-select">Playback Mode</label>
                <select id="playlist-mode-select">
                    <option value="sequential">Sequential</option>
                    <option value="loop">Loop</option>
                    <option value="shuffle">Shuffle</option>
                </select>
            </div>
            <div class="form-group" style="display: flex; align-items: center; gap: 8px;">
                <input type="checkbox" id="playlist-clearing-toggle" checked style="width: auto;">
                <label for="playlist-clearing-toggle" style="margin-bottom: 0;">Run clearing pattern between items</label>
            </div>
            <div class="button-group" style="margin-bottom: 16px;">
                <button class="btn-primary" id="btn-playlist-start">▶ Start Playlist</button>
                <button class="btn-danger" id="btn-playlist-stop">⏹ Stop</button>
            </div>
            <div id="playlist-items" style="margin-bottom: 16px;">
                <p style="text-align:center; color:#718096;">Playlist is empty</p>
            </div>
            <div class="button-group">
                <button class="btn-secondary" id="btn-add-to-playlist">+ Add Current Pattern</button>
                <button class="btn-primary" id="btn-add-all-to-playlist">+ Add All Patterns</button>
                <button class="btn-secondary" id="btn-clear-playlist">Clear All</button>
            </div>
            <div style="margin-top: 16px;">
                <div class="form-group">
                    <label for="playlist-name-input">Playlist Name</label>
                    <input type="text" id="playlist-name-input" placeholder="my-playlist" style="width: 100%; padding: 8px; border: 1px solid #e2e8f0; border-radius: 4px;">
                </div>
                <div class="button-group">
                    <button class="btn-secondary" id="btn-save-playlist">💾 Save</button>
                    <button class="btn-secondary" id="btn-load-playlist">📂 Load</button>
                </div>
            </div>
        </div>

        <div class="card">
            <h2>File Management</h2>
            <div class="upload-area" id="upload-area">
                <p>Click to upload .thr file or drag and drop</p>
                <input type="file" id="file-input" accept=".thr" style="display:none">
            </div>
            <div class="file-list" id="file-list">
                <p style="text-align:center; color:#718096;">Loading files...</p>
            </div>
        </div>

        <div class="card">
            <h2>System Information</h2>
            <div class="system-info" id="system-info">
                <div class="info-item">
                    <span>Free Heap:</span>
                    <span id="heap">-</span>
                </div>
                <div class="info-item">
                    <span>WiFi SSID:</span>
                    <span id="wifi-ssid">-</span>
                </div>
                <div class="info-item">
                    <span>IP Address:</span>
                    <span id="wifi-ip">-</span>
                </div>
                <div class="info-item">
                    <span>Signal:</span>
                    <span id="wifi-rssi">-</span>
                </div>
            </div>
        </div>

        <div class="card">
            <div class="console-header">
                <h2 style="margin-bottom: 0; border-bottom: none; padding-bottom: 0;">Console</h2>
                <div>
                    <span id="console-status" class="console-status console-disconnected">Disconnected</span>
                    <button class="btn-small btn-secondary" id="btn-clear-console" style="margin-left: 8px;">Clear</button>
                </div>
            </div>
            <div class="console-container" id="console-output">
                <div class="console-line">Connecting to console...</div>
            </div>
        </div>
    </div>

    <script>
        class SisyphusController {
            constructor() {
                this.apiBase = '/api';
                this.statusInterval = null;
                this.positionInterval = null;
                this.canvas = null;
                this.ctx = null;
                this.init();
            }

            async init() {
                this.setupCanvas();
                this.setupEventListeners();
                this.setupConsole();
                await this.loadFileList();
                await this.loadSystemInfo();
                await this.loadPlaylistStatus();
                this.startStatusPolling();
                this.startPositionPolling();
            }

            setupConsole() {
                this.consoleOutput = document.getElementById('console-output');
                this.consoleStatus = document.getElementById('console-status');
                this.maxConsoleLines = 200;

                document.getElementById('btn-clear-console').addEventListener('click', () => {
                    this.consoleOutput.innerHTML = '';
                });

                this.connectConsole();
            }

            connectConsole() {
                this.eventSource = new EventSource('/api/console');

                this.eventSource.onopen = () => {
                    this.consoleStatus.textContent = 'Connected';
                    this.consoleStatus.className = 'console-status console-connected';
                };

                this.eventSource.onerror = () => {
                    this.consoleStatus.textContent = 'Disconnected';
                    this.consoleStatus.className = 'console-status console-disconnected';
                    // Try to reconnect after 2 seconds
                    setTimeout(() => this.connectConsole(), 2000);
                };

                this.eventSource.addEventListener('log', (e) => {
                    this.appendToConsole(e.data);
                });
            }

            appendToConsole(text) {
                // Split by newlines and add each line
                const lines = text.split('\n');
                for (const line of lines) {
                    if (line.trim() === '') continue;
                    const div = document.createElement('div');
                    div.className = 'console-line';
                    div.textContent = line;
                    this.consoleOutput.appendChild(div);
                }

                // Limit number of lines
                while (this.consoleOutput.children.length > this.maxConsoleLines) {
                    this.consoleOutput.removeChild(this.consoleOutput.firstChild);
                }

                // Auto-scroll to bottom
                this.consoleOutput.scrollTop = this.consoleOutput.scrollHeight;
            }

            setupCanvas() {
                this.canvas = document.getElementById('position-canvas');
                this.ctx = this.canvas.getContext('2d');
                // Initial draw of table
                this.drawTable();
            }

            setupEventListeners() {
                document.getElementById('brightness-slider').addEventListener('input', (e) => {
                    const value = e.target.value;
                    document.getElementById('brightness-value').textContent = value + '%';
                });

                document.getElementById('brightness-slider').addEventListener('change', (e) => {
                    this.setBrightness(parseInt(e.target.value));
                });

                document.getElementById('btn-led-off').addEventListener('click', () => {
                    this.setBrightness(0);
                    document.getElementById('brightness-slider').value = 0;
                    document.getElementById('brightness-value').textContent = '0%';
                });

                document.getElementById('speed-slider').addEventListener('input', (e) => {
                    const value = e.target.value;
                    document.getElementById('speed-value').textContent = value;
                });

                document.getElementById('speed-slider').addEventListener('change', (e) => {
                    this.setSpeed(parseInt(e.target.value));
                });

                document.getElementById('btn-start').addEventListener('click', () => this.startPattern());
                document.getElementById('btn-pause').addEventListener('click', () => this.pausePattern());
                document.getElementById('btn-stop').addEventListener('click', () => this.stopPattern());
                document.getElementById('btn-home').addEventListener('click', () => this.homeDevice());

                // Playlist event listeners
                document.getElementById('btn-add-to-playlist').addEventListener('click', () => this.addToPlaylist());
                document.getElementById('btn-add-all-to-playlist').addEventListener('click', () => this.addAllToPlaylist());
                document.getElementById('btn-clear-playlist').addEventListener('click', () => this.clearPlaylist());
                document.getElementById('btn-playlist-start').addEventListener('click', () => this.startPlaylist());
                document.getElementById('btn-playlist-stop').addEventListener('click', () => this.stopPlaylist());
                document.getElementById('btn-save-playlist').addEventListener('click', () => this.savePlaylist());
                document.getElementById('btn-load-playlist').addEventListener('click', () => this.loadPlaylist());
                document.getElementById('playlist-mode-select').addEventListener('change', (e) => this.setPlaylistMode(e.target.value));
                document.getElementById('playlist-clearing-toggle').addEventListener('change', (e) => this.setPlaylistClearing(e.target.checked));

                const uploadArea = document.getElementById('upload-area');
                const fileInput = document.getElementById('file-input');

                uploadArea.addEventListener('click', () => fileInput.click());
                fileInput.addEventListener('change', (e) => this.uploadFile(e.target.files[0]));

                uploadArea.addEventListener('dragover', (e) => {
                    e.preventDefault();
                    uploadArea.style.borderColor = '#667eea';
                });

                uploadArea.addEventListener('dragleave', () => {
                    uploadArea.style.borderColor = '#cbd5e0';
                });

                uploadArea.addEventListener('drop', (e) => {
                    e.preventDefault();
                    uploadArea.style.borderColor = '#cbd5e0';
                    if (e.dataTransfer.files.length > 0) {
                        this.uploadFile(e.dataTransfer.files[0]);
                    }
                });
            }

            async getStatus() {
                const response = await fetch(this.apiBase + '/status');
                return await response.json();
            }

            async getPosition() {
                const response = await fetch(this.apiBase + '/position');
                return await response.json();
            }

            drawTable() {
                const ctx = this.ctx;
                const centerX = this.canvas.width / 2;
                const centerY = this.canvas.height / 2;
                const radius = Math.min(centerX, centerY) - 20;

                // Clear canvas
                ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

                // Draw table circle
                ctx.strokeStyle = '#cbd5e0';
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
                ctx.stroke();

                // Draw center point
                ctx.fillStyle = '#cbd5e0';
                ctx.beginPath();
                ctx.arc(centerX, centerY, 3, 0, 2 * Math.PI);
                ctx.fill();
            }

            drawPosition(data) {
                this.drawTable();

                const ctx = this.ctx;
                const centerX = this.canvas.width / 2;
                const centerY = this.canvas.height / 2;
                const radius = Math.min(centerX, centerY) - 20;

                // Draw path
                if (data.path && data.path.length > 1) {
                    ctx.strokeStyle = '#667eea';
                    ctx.lineWidth = 2;
                    ctx.beginPath();
                    for (let i = 0; i < data.path.length; i++) {
                        const point = data.path[i];
                        const x = centerX + (point.x - 0.5) * 2 * radius;
                        const y = centerY + (point.y - 0.5) * 2 * radius;
                        if (i === 0) {
                            ctx.moveTo(x, y);
                        } else {
                            ctx.lineTo(x, y);
                        }
                    }
                    ctx.stroke();
                }

                // Draw current position
                if (data.current) {
                    const x = centerX + (data.current.x - 0.5) * 2 * radius;
                    const y = centerY + (data.current.y - 0.5) * 2 * radius;

                    ctx.fillStyle = '#48bb78';
                    ctx.beginPath();
                    ctx.arc(x, y, 8, 0, 2 * Math.PI);
                    ctx.fill();

                    // Draw ball outline
                    ctx.strokeStyle = '#38a169';
                    ctx.lineWidth = 2;
                    ctx.stroke();

                    // Update coordinate display
                    const coordsElement = document.getElementById('position-coords');
                    const rho = data.current.rho.toFixed(1);
                    let displayTheta = (data.current.theta * 180 / Math.PI) % 360;
                    if (displayTheta < 0) displayTheta += 360;
                    const theta = displayTheta.toFixed(1);
                    const cartX = ((data.current.x - 0.5) * 2 * 450).toFixed(1); // Convert back to mm from center
                    const cartY = ((data.current.y - 0.5) * 2 * 450).toFixed(1);
                    coordsElement.textContent = `ρ: ${rho}mm, θ: ${theta}°, X: ${cartX}mm, Y: ${cartY}mm`;
                } else {
                    document.getElementById('position-coords').textContent = 'Position: Not available';
                }
            }

            startPositionPolling() {
                this.positionInterval = setInterval(async () => {
                    try {
                        const position = await this.getPosition();
                        this.drawPosition(position);
                    } catch (error) {
                        // Silently handle errors to avoid console spam
                    }
                }, 500); // Update 2 times per second
            }

            async startPattern() {
                const file = document.getElementById('pattern-select').value;
                const clearing = document.getElementById('clearing-select').value;

                if (!file) {
                    alert('Please select a pattern file');
                    return;
                }

                const formData = new FormData();
                formData.append('file', file);
                formData.append('clearing', clearing);

                const response = await fetch(this.apiBase + '/pattern/start', {
                    method: 'POST',
                    body: formData
                });

                const result = await response.json();
                if (!result.success) {
                    alert('Error: ' + result.message);
                }
            }

            async stopPattern() {
                await fetch(this.apiBase + '/pattern/stop', { method: 'POST' });
            }

            async pausePattern() {
                await fetch(this.apiBase + '/pattern/pause', { method: 'POST' });
            }

            async resumePattern() {
                await fetch(this.apiBase + '/pattern/resume', { method: 'POST' });
            }

            async homeDevice() {
                if (!confirm('Home the device? This will reset the position to center.')) {
                    return;
                }

                const response = await fetch(this.apiBase + '/home', { method: 'POST' });
                const result = await response.json();

                if (result.success) {
                    alert('Device homed successfully');
                } else {
                    alert('Failed to home device: ' + (result.message || 'Unknown error'));
                }
            }

            async setBrightness(value) {
                const formData = new FormData();
                formData.append('brightness', value);

                await fetch(this.apiBase + '/led/brightness', {
                    method: 'POST',
                    body: formData
                });
            }

            async setSpeed(value) {
                const formData = new FormData();
                formData.append('speed', value);

                await fetch(this.apiBase + '/speed', {
                    method: 'POST',
                    body: formData
                });
            }

            async loadFileList() {
                const response = await fetch(this.apiBase + '/files');
                const data = await response.json();

                const select = document.getElementById('pattern-select');
                const fileList = document.getElementById('file-list');

                select.innerHTML = '<option value="">Select a pattern...</option>';
                fileList.innerHTML = '';

                if (data.files && data.files.length > 0) {
                    data.files.forEach(file => {
                        const option = document.createElement('option');
                        option.value = file.name;
                        option.textContent = file.name;
                        select.appendChild(option);

                        const fileItem = document.createElement('div');
                        fileItem.className = 'file-item';
                        fileItem.innerHTML = `
                            <div class="file-info">
                                <div class="file-name">${file.name}</div>
                                <div class="file-size">${(file.size / 1024).toFixed(1)} KB</div>
                            </div>
                            <button class="btn-danger btn-small" onclick="controller.deleteFile('${file.name}')">Delete</button>
                        `;
                        fileList.appendChild(fileItem);
                    });
                } else {
                    fileList.innerHTML = '<p style="text-align:center; color:#718096;">No pattern files found</p>';
                }
            }

            async deleteFile(filename) {
                if (!confirm('Delete ' + filename + '?')) return;

                const formData = new FormData();
                formData.append('file', filename);

                await fetch(this.apiBase + '/files/delete', {
                    method: 'POST',
                    body: formData
                });

                await this.loadFileList();
            }

            async uploadFile(file) {
                if (!file) return;

                if (!file.name.endsWith('.thr')) {
                    alert('Only .thr files are allowed');
                    return;
                }

                const formData = new FormData();
                formData.append('file', file);

                const uploadArea = document.getElementById('upload-area');
                uploadArea.classList.add('loading');

                try {
                    const response = await fetch(this.apiBase + '/files/upload', {
                        method: 'POST',
                        body: formData
                    });

                    const result = await response.json();
                    if (result.success) {
                        await this.loadFileList();
                    } else {
                        alert('Upload failed: ' + result.message);
                    }
                } finally {
                    uploadArea.classList.remove('loading');
                }
            }

            async loadSystemInfo() {
                const response = await fetch(this.apiBase + '/system/info');
                const data = await response.json();

                document.getElementById('heap').textContent = Math.round(data.heap / 1024) + ' KB';
                document.getElementById('wifi-ssid').textContent = data.wifi.ssid;
                document.getElementById('wifi-ip').textContent = data.wifi.ip;
                document.getElementById('wifi-rssi').textContent = data.wifi.rssi + ' dBm';
            }

            updateUI(status) {
                const stateBadge = document.getElementById('state-badge');
                stateBadge.textContent = status.state;
                stateBadge.className = 'status-badge status-' + status.state.toLowerCase();

                document.getElementById('current-pattern').textContent = status.currentPattern || 'None';
                document.getElementById('status-brightness').textContent = status.ledBrightness;
                document.getElementById('uptime').textContent = this.formatUptime(status.uptime);

                const slider = document.getElementById('brightness-slider');
                if (document.activeElement !== slider) {
                    slider.value = status.ledBrightness;
                    document.getElementById('brightness-value').textContent = status.ledBrightness + '%';
                }
            }

            formatUptime(seconds) {
                const h = Math.floor(seconds / 3600);
                const m = Math.floor((seconds % 3600) / 60);
                const s = seconds % 60;

                if (h > 0) return `${h}h ${m}m`;
                if (m > 0) return `${m}m ${s}s`;
                return `${s}s`;
            }

            startStatusPolling() {
                this.statusInterval = setInterval(async () => {
                    const status = await this.getStatus();
                    this.updateUI(status);
                }, 1000);
            }

            // Playlist methods
            async addToPlaylist() {
                const file = document.getElementById('pattern-select').value;
                const clearing = document.getElementById('clearing-select').value;

                if (!file) {
                    alert('Please select a pattern first');
                    return;
                }

                const formData = new FormData();
                formData.append('file', file);
                formData.append('clearing', clearing);
                formData.append('useClearing', 'true');

                const response = await fetch(this.apiBase + '/playlist/add', {
                    method: 'POST',
                    body: formData
                });

                if (response.ok) {
                    await this.loadPlaylistStatus();
                }
            }

            async addAllToPlaylist() {
                if (!confirm('Add all patterns to playlist? This will add all .thr files.')) return;

                const response = await fetch(this.apiBase + '/playlist/addall', {
                    method: 'POST'
                });

                const result = await response.json();
                if (response.ok && result.success) {
                    alert(`Added ${result.count} patterns to playlist`);
                    await this.loadPlaylistStatus();
                } else {
                    alert('Failed to add patterns');
                }
            }

            async clearPlaylist() {
                if (!confirm('Clear all items from playlist?')) return;

                await fetch(this.apiBase + '/playlist/clear', { method: 'POST' });
                await this.loadPlaylistStatus();
            }

            async startPlaylist() {
                const response = await fetch(this.apiBase + '/playlist/start', { method: 'POST' });
                const result = await response.json();

                if (!result.success) {
                    alert('Error: ' + result.message);
                }
            }

            async stopPlaylist() {
                await fetch(this.apiBase + '/playlist/stop', { method: 'POST' });
            }

            async setPlaylistMode(mode) {
                const formData = new FormData();
                formData.append('mode', mode);

                await fetch(this.apiBase + '/playlist/mode', {
                    method: 'POST',
                    body: formData
                });
            }

            async setPlaylistClearing(enabled) {
                const formData = new FormData();
                formData.append('enabled', enabled ? 'true' : 'false');

                await fetch(this.apiBase + '/playlist/clearing', {
                    method: 'POST',
                    body: formData
                });
            }

            async savePlaylist() {
                const name = document.getElementById('playlist-name-input').value.trim();
                if (!name) {
                    alert('Please enter a playlist name');
                    return;
                }

                const formData = new FormData();
                formData.append('name', name);

                const response = await fetch(this.apiBase + '/playlist/save', {
                    method: 'POST',
                    body: formData
                });

                const result = await response.json();
                if (result.success) {
                    alert('Playlist saved: ' + name);
                } else {
                    alert('Error saving playlist');
                }
            }

            async loadPlaylist() {
                const name = document.getElementById('playlist-name-input').value.trim();
                if (!name) {
                    alert('Please enter a playlist name');
                    return;
                }

                const formData = new FormData();
                formData.append('name', name);

                const response = await fetch(this.apiBase + '/playlist/load', {
                    method: 'POST',
                    body: formData
                });

                if (response.ok) {
                    await this.loadPlaylistStatus();
                    alert('Playlist loaded: ' + name);
                } else {
                    alert('Playlist not found');
                }
            }

            async loadPlaylistStatus() {
                const response = await fetch(this.apiBase + '/playlist');
                const data = await response.json();

                // Update mode select
                document.getElementById('playlist-mode-select').value = data.mode;

                // Update clearing toggle
                document.getElementById('playlist-clearing-toggle').checked = data.clearingEnabled !== false;

                // Update playlist items display
                const container = document.getElementById('playlist-items');

                if (data.items && data.items.length > 0) {
                    container.innerHTML = data.items.map((item, index) => `
                        <div class="file-item" style="margin-bottom: 8px;">
                            <div class="file-info">
                                <div class="file-name">${index + 1}. ${item.filename}</div>
                                <div class="file-size" style="font-size: 12px; color: #718096;">
                                    ${item.clearing}${item.useClearing ? ' (with clearing)' : ' (no clearing)'}
                                </div>
                            </div>
                            <button class="btn-danger" style="padding: 4px 8px; font-size: 12px;"
                                    onclick="controller.removeFromPlaylist(${index})">✕</button>
                        </div>
                    `).join('');
                } else {
                    container.innerHTML = '<p style="text-align:center; color:#718096;">Playlist is empty</p>';
                }
            }

            async removeFromPlaylist(index) {
                const formData = new FormData();
                formData.append('index', index);

                await fetch(this.apiBase + '/playlist/remove', {
                    method: 'POST',
                    body: formData
                });

                await this.loadPlaylistStatus();
            }
        }

        const controller = new SisyphusController();
    </script>
</body>
</html>
)rawliteral";
