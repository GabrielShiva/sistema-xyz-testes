class ControlGUI {
    constructor() {
        this.port = null;
        this.writer = null;
        this.isConnected = false;
        this.reader = null;
        
        // Homing
        this.motorHomingStatus = [false, false, false];
        this.limitSwitchStatus = [false, false, false];
        
        // Estado do sistema de controle
        this.previousState = 'unknown';
        this.currentState = 'command';
        this.motorCount = 3;
        this.motorPositions = [0, 0, 0];
        this.savedPositions = [];

        this.keyboardDataToSave = [];
    
        // Carrega os teclados
        this.loadKeyboardData();
        this.initializeEventListeners();
        this.updateConnectionStatus();
        this.updateStateStatus();

        this.addLogEntry('Interface inicializada.', 'info');
    }

    async loadKeyboardData() {
        try {
            const response = await fetch('./teclados.json');
            this.keyboardDataToSave = await response.json();
            
            this.addLogEntry("Dados de teclado carregados.", "success");

            this.populateKeyboardSelect();
            // this.renderKeyboardKeys();    

        } catch (error) {
            console.error("Erro ao carregar JSON:", error);
            this.addLogEntry("Erro ao carregar dados de teclado!", "error");
        }
    }

    populateKeyboardSelect() {
        const select = document.getElementById('keyboard-select');
        const selectSec = document.getElementById('keyboard-select-sec');

        if (select) {
            // Limpa o select e adiciona a opção padrão
            select.innerHTML = `
                <option value="">Nenhum teclado selecionado</option>
            `;

            // Adiciona os teclados carregados do JSON
            this.keyboardDataToSave.forEach(kb => {
                const option = document.createElement('option');
                option.value = kb.name;        // o valor é o nome do teclado
                option.textContent = kb.name;  // o texto exibido também
                select.appendChild(option);
            });
        }

        if (selectSec) {
            // Limpa o select e adiciona a opção padrão
            selectSec.innerHTML = `
                <option value="">Nenhum teclado selecionado</option>
            `;

            // Adiciona os teclados carregados do JSON
            this.keyboardDataToSave.forEach(kb => {
                const option = document.createElement('option');
                option.value = kb.name;        // o valor é o nome do teclado
                option.textContent = kb.name;  // o texto exibido também
                selectSec.appendChild(option);
            });
        }

        // Log opcional
        this.addLogEntry(`Foram carregados ${this.keyboardDataToSave.length} teclados.`, 'info');
    }

    updateCharts(xValue, yValue) {
        // Adicionar novos dados
        const timestamp = this.chartDataX.length;
        
        this.chartDataX.push({ x: timestamp, y: xValue });
        this.chartDataY.push({ x: timestamp, y: yValue });

        // Manter apenas os últimos N pontos
        if (this.chartDataX.length > 30) {
            this.chartDataX.shift();
            this.chartDataY.shift();
            
            // Reindexar os dados
            this.chartDataX = this.chartDataX.map((point, index) => ({ x: index, y: point.y }));
            this.chartDataY = this.chartDataY.map((point, index) => ({ x: index, y: point.y }));
        }

        // Atualizar os gráficos
        this.chartX.updateSeries([{
            data: this.chartDataX
        }]);

        this.chartY.updateSeries([{
            data: this.chartDataY
        }]);
    }

    initializeEventListeners() {
        // Botões para conectar/desconectar da placa
        document.getElementById('connect-btn').addEventListener('click', () => this.connectToBoard());
        document.getElementById('disconnect-btn').addEventListener('click', () => this.disconnectFromBoard());
        document.getElementById('clear-log-btn').addEventListener('click', () => this.clearLog());

        // Botões de controle de modo de operação
        document.getElementById('joystick-mode-btn').addEventListener('click', () => this.setJoystickMode());
        document.getElementById('command-mode-btn').addEventListener('click', () => this.setCommandMode());
        document.getElementById('homming-mode-btn').addEventListener('click', () => this.setHomingMode());
        
        // Botões para controle de motores
        document.getElementById('move-forward-btn').addEventListener('click', () => this.moveForward());
        document.getElementById('move-backward-btn').addEventListener('click', () => this.moveBackward());
        document.getElementById('emergency-stop-btn').addEventListener('click', () => this.emergencyStop());
        document.getElementById('set-speed-btn').addEventListener('click', () => this.setSpeed());

        // Botões de referenciamento
        document.getElementById('home-all-btn').addEventListener('click', () => this.homeAllMotors());
        document.getElementById('home-x-btn').addEventListener('click', () => this.homeMotor(0));
        document.getElementById('home-y-btn').addEventListener('click', () => this.homeMotor(1));
        document.getElementById('home-z-btn').addEventListener('click', () => this.homeMotor(2));

        // Dual motor position control buttons
        document.getElementById('move-both-btn').addEventListener('click', () => this.moveBothMotors());
        document.getElementById('move-motor0-btn').addEventListener('click', () => this.moveMotor(0));
        document.getElementById('move-motor1-btn').addEventListener('click', () => this.moveMotor(1));
        document.getElementById('move-motor2-btn').addEventListener('click', () => this.moveMotor(2));
        document.getElementById('set-zero-all-btn').addEventListener('click', () => this.setZeroAllMotors());
        document.getElementById('set-zero-motor0-btn').addEventListener('click', () => this.setZeroMotor(0));
        document.getElementById('set-zero-motor1-btn').addEventListener('click', () => this.setZeroMotor(1));
        document.getElementById('set-zero-motor2-btn').addEventListener('click', () => this.setZeroMotor(2));
   
        // Position memory system buttons
        document.getElementById('add-keyboard-btn').addEventListener('click', () => this.addKeyboard());
        document.getElementById('keyboard-select').addEventListener('change', (event) => {
            if (event.target.value === '') {
                document.getElementById('position-label-input').disabled = true;
                document.getElementById('save-position-btn').disabled = true;
            } else {
                document.getElementById('position-label-input').disabled = false;
                document.getElementById('save-position-btn').disabled = false;
            }
        });
        document.getElementById('save-position-btn').addEventListener('click', () => this.saveCurrentPosition());
        document.getElementById('keyboard-select').addEventListener('change', () => {
            this.renderKeyboardKeys();
        });
        document.getElementById('keyboard-select-sec').addEventListener('change', () => {
            this.renderKeyboardKeys(0);
        });
        // document.getElementById('recall-position-btn').addEventListener('click', () => this.recallPosition());
        // document.getElementById('clear-positions-btn').addEventListener('click', () => this.clearAllPositions());
        // document.getElementById('test-all-positions-btn').addEventListener('click', () => this.testAllPositions());
        // document.getElementById('send-string-btn').addEventListener('click', () => this.sendStringToTest());
    }

    // Conectar e desconectar da placa
    async connectToBoard() {
        try {
            // verifica se o navegador contém a SerialAPI
            if (!navigator.serial) {
                // caso não tenha, exibe mensagem de erro na interface
                return;
            }

            this.port = await navigator.serial.requestPort();
            await this.port.open({ baudRate: 115200 }); // abre a porta com um baudRate

            this.writer = this.port.writable.getWriter(); // utilizado para escrever na linha de comunicação
            this.isConnected = true; // define que a conexão foi estabelecida

            // atualiza a tela indicando que a conexão foi estabelecida
            this.updateConnectionStatus();
            this.updateStateStatus();


            // Mantem leitura do barramento de dados
            this.startReading();

            // Envia comando para a placa indicando início de conexão
            this.sendCommand('START_CONN');
        } catch (error) {
          console.log('error!');  
        }
    }

    async disconnectFromBoard() {
        if (this.reader) {
            await this.reader.cancel();
        }
        if (this.writer) {
            await this.writer.close();
        }
        if (this.port) {
            await this.port.close();
        }

        this.port = null;
        this.reader = null;
        this.writer = null;
        this.isConnected = false;
        this.currentState = 'unknown';
        this.updateConnectionStatus();
        this.updateStateStatus();
        this.addLogEntry('Conexão com a placa foi fechada', 'info');
    }

    // Controle de estado da interface
    updateConnectionStatus() {
        const statusIndicator = document.getElementById('connection-status');
        const statusText = document.getElementById('connection-text');
        const connectBtn = document.getElementById('connect-btn');
        const disconnectBtn = document.getElementById('disconnect-btn');

        if (this.isConnected) {
            statusIndicator.className = 'status-indicator status-connected';
            statusText.textContent = 'Conectado';
            connectBtn.disabled = true;
            disconnectBtn.disabled = false;

            document.querySelector('.control-panel').classList.remove('control-panel-disconected');
            document.querySelector('.info-block').classList.remove('info-block-active');

            // document.querySelectorAll('.section:not(#control-panel-header .section)')
            //     .forEach(section => {
            //         if (section.id !== 'joystick-readings') {
            //             section.classList.add('section-active');
            //         }
                    
            //     });
        } else {
            statusIndicator.className = 'status-indicator status-disconnected';
            statusText.textContent = 'Desconectado';
            connectBtn.disabled = false;
            disconnectBtn.disabled = true;

            document.querySelector('.control-panel').classList.add('control-panel-disconected');
            document.querySelector('.info-block').classList.add('info-block-active');

            // document.querySelectorAll('.section:not(#control-panel-header .section)')
            //     .forEach(section => {
            //         section.classList.remove('section-active');
            //     });
        }
    }

    updateStateStatus() {
        const stateIndicator = document.getElementById('state-indicator');
        const stateText = document.getElementById('state-text');
        let joystickBtn = document.getElementById('joystick-mode-btn');
        let commandBtn = document.getElementById('command-mode-btn');
        let hommingBtn = document.getElementById('homming-mode-btn');
        let disconectBtn = document.getElementById('disconnect-btn');
        const motorControlButtons = document.querySelectorAll('#move-forward-btn, #move-backward-btn, #set-speed-btn, #emergency-stop-btn');
        const positionControlButtons = document.querySelectorAll('#move-both-btn, #move-motor0-btn, #move-motor1-btn, #move-motor2-btn, #set-zero-all-btn, #set-zero-motor0-btn, #set-zero-motor1-btn, #set-zero-motor2-btn');
        const rotuloButtons = document.querySelectorAll('#clear-positions-btn, #recall-position-btn, #test-all-positions-btn, #send-string-btn');
        const memoryControlButtons = document.querySelectorAll('#recall-position-btn');
        const homingControlButtons = document.querySelectorAll('#home-all-btn, #home-x-btn, #home-y-btn, #home-z-btn');

        switch(this.currentState) {
            case 'joystick':
                stateIndicator.className = 'status-indicator status-connected';
                stateText.textContent = `Modo de Mapeamento (${this.motorCount} motor${this.motorCount > 1 ? 'es' : ''})`;
                joystickBtn.disabled = true;
                commandBtn.disabled = false;
                hommingBtn.disabled = false;
                motorControlButtons.forEach(btn => btn.disabled = true);
                positionControlButtons.forEach(btn => btn.disabled = true);
                memoryControlButtons.forEach(btn => btn.disabled = true);
                rotuloButtons.forEach(btn => btn.disabled = false);
                disconectBtn.disabled = false;  

                document.querySelector('.info-block').classList.remove('info-block-active');

                // painel esquerdo
                document.querySelector('#joystick-readings').classList.add('section-active');
                document.querySelector('#axis-readings').classList.add('section-active');
                document.querySelector('#log-readings').classList.add('section-active');

                // painel direito
                document.querySelector('#homing-section').classList.remove('section-active');
                document.querySelector('#new-keyboard-section').classList.add('section-active');
                document.querySelector('#new-positions-section').classList.add('section-active');
                document.querySelector('#keyboard-test-section').classList.remove('section-active');

                this.updateKeyboardSelect();
                this.removeKeys();
                break;
            case 'command':
                stateIndicator.className = 'status-indicator status-connected';
                stateText.textContent = `Modo de Comando (${this.motorCount} motor${this.motorCount > 1 ? 'es' : ''})`;
                joystickBtn.disabled = false;
                commandBtn.disabled = true;
                hommingBtn.disabled = false;
                motorControlButtons.forEach(btn => btn.disabled = false);
                positionControlButtons.forEach(btn => btn.disabled = false);
                memoryControlButtons.forEach(btn => btn.disabled = false);
                homingControlButtons.forEach(btn => btn.disabled = true);
                rotuloButtons.forEach(btn => btn.disabled = false);
                disconectBtn.disabled = false;  

                document.querySelector('.info-block').classList.remove('info-block-active');

                // painel esquerdo
                document.querySelector('#joystick-readings').classList.remove('section-active');
                document.querySelector('#axis-readings').classList.add('section-active');
                document.querySelector('#log-readings').classList.add('section-active');

                // painel direito
                document.querySelector('#homing-section').classList.remove('section-active');
                document.querySelector('#new-keyboard-section').classList.remove('section-active');
                document.querySelector('#new-positions-section').classList.remove('section-active');
                document.querySelector('#keyboard-test-section').classList.add('section-active');

                this.updateKeyboardSelect(0);
                break;
            case 'homing':
                stateIndicator.className = 'status-indicator status-homing';
                stateText.textContent = `Homing (${this.motorCount} motor${this.motorCount > 1 ? 'es' : ''})`;
                joystickBtn.disabled = false;
                commandBtn.disabled = false;
                hommingBtn.disabled = true;
                motorControlButtons.forEach(btn => btn.disabled = true);
                positionControlButtons.forEach(btn => btn.disabled = true);
                memoryControlButtons.forEach(btn => btn.disabled = true);
                homingControlButtons.forEach(btn => btn.disabled = false);
                rotuloButtons.forEach(btn => btn.disabled = true);
                document.getElementById('emergency-stop-btn').disabled = true;

                document.querySelector('.info-block').classList.remove('info-block-active');
                
                // painel esquerdo
                document.querySelector('#joystick-readings').classList.remove('section-active');
                document.querySelector('#axis-readings').classList.add('section-active');
                document.querySelector('#log-readings').classList.add('section-active');

                // painel direito
                document.querySelector('#homing-section').classList.add('section-active');
                document.querySelector('#new-keyboard-section').classList.remove('section-active');
                document.querySelector('#new-positions-section').classList.remove('section-active');
                document.querySelector('#keyboard-test-section').classList.remove('section-active');
                break;
            default:
                stateIndicator.className = 'status-indicator status-disconnected';
                stateText.textContent = 'Desconhecido';
                joystickBtn.disabled = true;     
                commandBtn.disabled = true;  
                hommingBtn.disabled = true;
                disconectBtn.disabled = true;
                motorControlButtons.forEach(btn => btn.disabled = true);
                positionControlButtons.forEach(btn => btn.disabled = true);
                memoryControlButtons.forEach(btn => btn.disabled = true);
                rotuloButtons.forEach(btn => btn.disabled = false);
                homingControlButtons.forEach(btn => btn.disabled = true);

                document.querySelector('.info-block').classList.add('info-block-active');
        }
    }

    updateHomingStatusDisplay() {
        const container = document.getElementById('homing-status-grid');
        container.innerHTML = '';

        for (let i = 0; i < this.motorCount && i < 3; i++) {
            const axisLabel = i === 0 ? 'X' : i === 1 ? 'Y' : 'Z';
            const colorClass = i === 0 ? 'axis-color-x' : i === 1 ? 'axis-color-y' : '';

            const homingItem = document.createElement('div');
            homingItem.className = 'homing-status-item';

            const homingStatus = this.motorHomingStatus[i] ? 'REFERENCIADO' : 'NÃO REFERENCIADO';
            const homingClass = this.motorHomingStatus[i] ? 'homed' : 'not-homed';
            const limitClass = this.limitSwitchStatus[i] ? 'limit-switch-triggered' : 'limit-switch-clear';

            homingItem.innerHTML = `
                <div class="homing-status-label ${colorClass}">Eixo ${axisLabel}</div>
                <div class="homing-status-value ${homingClass}">${homingStatus}</div>
                <div class="homing-status-label" style='margin-top: 15px;'>Fim de Curso</div>
                <div class="homing-status-value">
                    <span class="limit-switch-indicator ${limitClass}"></span>
                    ${this.limitSwitchStatus[i] ? 'ATIVADO' : 'LIVRE'}
                </div>
            `;
            container.appendChild(homingItem);
        }
    }

    updateDataDisplay(xReading, yReading, motor0Dir, motor1Dir, motor0Interval, motor1Interval) {
        document.getElementById('joystick-x-value').textContent = xReading;
        document.getElementById('joystick-y-value').textContent = yReading;

        document.getElementById('motor0-dir-value').textContent = motor0Dir;
        document.getElementById('motor1-dir-value').textContent = motor1Dir;
        document.getElementById('motor0-interval-value').textContent = motor0Interval.toFixed(1);
        document.getElementById('motor1-interval-value').textContent = motor1Interval.toFixed(1);

        const motor0Speed = motor0Interval > 0 ? Math.round(1000000 / motor0Interval) : 0;
        const motor1Speed = motor1Interval > 0 ? Math.round(1000000 / motor1Interval) : 0;
        document.getElementById('motor0-speed-value').textContent = motor0Speed;
        document.getElementById('motor1-speed-value').textContent = motor1Speed;

        // Atualizar gráficos
        // this.updateCharts(xReading, yReading);
    }

    renderKeyboardKeys(opt = 1) {
        if (opt) {
            const container = document.getElementById('keyboard-keys-container');
            container.innerHTML = ''; // limpa tudo

            const keyboardId = document.getElementById('keyboard-select').value;
            const keyboardObj = this.keyboardDataToSave.find(k => k.name == keyboardId);

            if (!keyboardObj || !keyboardObj.keys || keyboardObj.keys.length === 0) {
                container.innerHTML = '<span>Nenhuma posição salva!</span>';
                return;
            }

            keyboardObj.keys.forEach(key => {
                const div = document.createElement('div');
                div.classList.add('keyboard-key');
                div.dataset.keyVal = key.character;

                div.innerHTML = `
                    <span class="keyboard-key-val">${key.character}</span>
                    <div>
                        <div class="keyboard-key-x-val">X: ${key.xPos}</div>
                        <div class="keyboard-key-y-val">Y: ${key.yPos}</div>
                    </div>
                    <button class="button danger remove-key-btn">X</button>
                `;

                container.appendChild(div);
            });
        } else {
            const container = document.getElementById('keyboard-keys-container-sec');
            container.innerHTML = ''; // limpa tudo

            const keyboardId = document.getElementById('keyboard-select-sec').value;
            const keyboardObj = this.keyboardDataToSave.find(k => k.name == keyboardId);

            if (!keyboardObj || !keyboardObj.keys || keyboardObj.keys.length === 0) {
                container.innerHTML = '<span>Nenhuma posição salva!</span>';
                return;
            }

            keyboardObj.keys.forEach(key => {
                const div = document.createElement('div');
                div.classList.add('keyboard-key');
                div.dataset.keyVal = key.character;

                div.innerHTML = `
                    <span class="keyboard-key-val">${key.character}</span>
                    <div>
                        <div class="keyboard-key-x-val">X: ${key.xPos}</div>
                        <div class="keyboard-key-y-val">Y: ${key.yPos}</div>
                    </div>
                    <button class="button primary test-key-btn" data-key-x="${key.xPos}" data-key-y="${key.yPos}">Testar</button>
                `;

                container.appendChild(div);
            });

            document.querySelectorAll('.test-key-btn').forEach(btn => {
                btn.addEventListener('click', () => {
                    const xPos = btn.dataset.keyX;
                    const yPos = btn.dataset.keyY;
                    // this.sendCommand(`TESTPOS,${xPos},${yPos}`);
                    this.sendCommand(`MOVETO,0,${xPos},1,${yPos},1`);
                });
            });
            
        }
    }

    // Leitura contínua e processamento dos dados de comunicação
    async startReading() {
        const textDecoder = new TextDecoderStream();
        const readableStreamClosed = this.port.readable.pipeTo(textDecoder.writable);
        this.reader = textDecoder.readable.getReader();

        let buffer = '';

        try {
            while (this.isConnected) {
                const { value, done } = await this.reader.read();
                if (done) break;

                buffer += value;
                const lines = buffer.split('\n');
                buffer = lines.pop();

                for (const line of lines) {
                    this.processDataLine(line.trim());
                }
            }
        } catch (error) {
            this.addLogEntry(`Erro de leitura: ${error.message}`, 'error');
        } finally {
            this.reader.releaseLock();
        }
    }

    processDataLine(line) {
        if (!line) return;

        if (line.startsWith('DATA,')) {
            try {
                const parts = line.split(',');
                //
                // New format: DATA,x_reading,y_reading,motor0_dir,motor1_dir,motor0_interval,motor1_interval
                if (parts.length === 7) {
                    const xReading = parseInt(parts[1]);
                    const yReading = parseInt(parts[2]);
                    const motor0Dir = parseInt(parts[3]);
                    const motor1Dir = parseInt(parts[4]);
                    const motor0Interval = parseFloat(parts[5]);
                    const motor1Interval = parseFloat(parts[6]);

                    // Only update display every few readings to reduce DOM updates
                    this.dataUpdateCounter = (this.dataUpdateCounter || 0) + 1;
                    if (this.dataUpdateCounter % 5 === 0) { // Update every 5th reading
                        this.updateDataDisplay(xReading, yReading, motor0Dir, motor1Dir, motor0Interval, motor1Interval);
                    }

                    this.latestXReading = xReading;
                    this.latestYReading = yReading;
                    this.latestMotor0Dir = motor0Dir;
                    this.latestMotor1Dir = motor1Dir;
                    this.latestMotor0Interval = motor0Interval;
                    this.latestMotor1Interval = motor1Interval;
                }
                // Fallback for old single-axis format: DATA,reading,dir,interval
                else if (parts.length === 4) {
                    const reading = parseInt(parts[1]);
                    const direction = parseInt(parts[2]);
                    const interval = parseFloat(parts[3]);

                    // Treat as single axis data (legacy support)
                    this.dataUpdateCounter = (this.dataUpdateCounter || 0) + 1;
                    if (this.dataUpdateCounter % 5 === 0) {
                        this.updateDataDisplay(reading, 0, direction, 0, interval, 0);
                    }

                    this.latestXReading = reading;
                    this.latestYReading = 0;
                    this.latestMotor0Dir = direction;
                    this.latestMotor1Dir = 0;
                    this.latestMotor0Interval = interval;
                    this.latestMotor1Interval = 0;
                }

            } catch (error) {
                this.addLogEntry(`Parse error: ${error.message}`, 'error');
            }
        } else if (line.startsWith('STATE,')) {
            try {
                const parts = line.split(',');
                if (parts.length >= 2) {
                    this.currentState = parts[1].toLowerCase();
                    if (parts.length >= 3) {
                        this.motorCount = parseInt(parts[2]);
                    }
                    this.updateStateStatus();
                }
            } catch (error) {
                this.addLogEntry(`Erro de parsing: ${error.message}`, 'error');
            }
        } else if (line.startsWith('POSITION,')) {
            try {
                const parts = line.split(',');
                // Format: POSITION,motor0_pos,motor1_pos,...
                for (let i = 1; i < parts.length && i <= this.motorCount; i++) {
                    this.motorPositions[i - 1] = parseInt(parts[i]);
                    const posElement = document.getElementById(`motor${i-1}-position`);
                    if (posElement) {
                        posElement.textContent = this.motorPositions[i - 1];
                    }
                }
            } catch (error) {
                this.addLogEntry(`Erro ao recuperar posição do motor: ${error.message}`, 'error');
            }
        } else if (line.startsWith('HOMING_STATUS,')) {
            try {
                const parts = line.split(',');
                // Format: HOMING_STATUS,motor0_homed,motor0_limit,motor1_homed,motor1_limit,...
                for (let i = 0; i < this.motorCount && i < 3; i++) {
                    const homedIndex = 1 + (i * 2);
                    const limitIndex = 2 + (i * 2);

                    if (homedIndex < parts.length) {
                        this.motorHomingStatus[i] = parseInt(parts[homedIndex]) === 1;
                    }
                    if (limitIndex < parts.length) {
                        this.limitSwitchStatus[i] = parseInt(parts[limitIndex]) === 1;
                    }
                }
                this.updateHomingStatusDisplay();
            } catch (error) {
                this.addLogEntry(`Homing status parse error: ${error.message}`, 'error');
            }
        } else if (line.startsWith('ACK,') || line.startsWith('ERROR,') || line.startsWith('WARNING,')) {
            const type = line.startsWith('ERROR,') ? 'error' : line.startsWith('WARNING,') ? 'warning' : 'success';
            this.addLogEntry(line, type);
        }
    }

    async sendCommand(command) {
        if (!this.isConnected || !this.writer) {
            this.addLogEntry('Não está conectado a nenhuma placa', 'error');
            return false;
        }

        try {
            const encoder = new TextEncoder();
            const data = encoder.encode(command + '\n');
            await this.writer.write(data);
            this.addLogEntry(`Enviado: ${command}`, 'info');
            return true;
        } catch (error) {
            this.addLogEntry(`Erro: ${error.message}`, 'error');
            return false;
        }
    }

    // Funções de controle de modo de operação
    async setJoystickMode() {
        this.previousState = this.currentState;
        await this.sendCommand('MODE,JOYSTICK');
    }

    async setCommandMode() {
        this.previousState = this.currentState;
        await this.sendCommand('MODE,COMMAND');
    }

    async setHomingMode() {
        this.previousState = this.currentState;
        await this.sendCommand('MODE,HOMING');
    }

    updateSavedPositionsDisplay() {
        const container = document.getElementById('saved-positions-container');

        if (this.savedPositions.length === 0) {
            container.innerHTML = '<div class="log-entry" style="color: #999;">No positions saved yet...</div>';
            return;
        }

        let html = '';
        for (const pos of this.savedPositions) {
            const colorClass = pos.character.match(/[0-9]/) ? 'info' : '';
            html += `<div class="log-entry ${colorClass}">` +
                `'${pos.character}' → X:${pos.x_position}, Y:${pos.y_position}</div>`;
        }

        container.innerHTML = html;
        container.scrollTop = container.scrollHeight;
    }

    // Controle individual dos motores de passo
    async moveForward() {
        const motorId = parseInt(document.getElementById('motor-id-select').value);
        const steps = parseInt(document.getElementById('steps-input').value) || 100;
        await this.sendCommand(`MOVE,${motorId},${steps}`);
    }

    async moveBackward() {
        const motorId = parseInt(document.getElementById('motor-id-select').value);
        const steps = parseInt(document.getElementById('steps-input').value) || 100;
        await this.sendCommand(`MOVE,${motorId},${-steps}`);
    }
    
    async setSpeed() {
        const speed = parseInt(document.getElementById('speed-input').value) || 1000;
        await this.sendCommand(`SPEED,${speed}`);
    }

    async emergencyStop() {
        await this.sendCommand('STOP');
    }

    // Funções de referenciamento à origem do eixo (X, Y e Z)
    async homeAllMotors() {
        if (confirm('Você deseja posicionar o atuador na origem dos três eixos?')) {
            await this.sendCommand('HOME');
        }
    }

    async homeMotor(motorId) {
        let motorName = '';

        if (motorId === 0) {
            motorName = 'X';
        }

        if (motorId === 1) {
            motorName = 'Y';
        }

        if (motorId === 2) {
            motorName = 'Z';
        }

        if (confirm(`Você deseja posicionar o atuador na origem do eixo ${motorName}?`)) {
            await this.sendCommand(`HOME,${motorId}`);
        }
    }

    // Controle de vários motores
    async moveBothMotors() {
        const pos0 = parseInt(document.getElementById('motor0-position-input').value) || 0;
        const pos1 = parseInt(document.getElementById('motor1-position-input').value) || 0;
        const waitForCompletion = document.getElementById('wait-completion-checkbox').checked ? 1 : 0;

        await this.sendCommand(`MOVETO,0,${pos0},1,${pos1},${waitForCompletion}`);
    }

    async moveMotor(motorId) {
        const posInput = document.getElementById(`motor${motorId}-position-input`);
        const position = parseInt(posInput.value) || 0;
        const waitForCompletion = document.getElementById('wait-completion-checkbox').checked ? 1 : 0;

        await this.sendCommand(`MOVETO,${motorId},${position},${waitForCompletion}`);
    }

    async setZeroAllMotors() {
        await this.sendCommand('SETZERO');
    }

    async setZeroMotor(motorId) {
        await this.sendCommand(`SETZERO,${motorId}`);
    }

    // Controle do sistema de posicionamento (palavras e letras)
    addKeyboard() {
        const input = document.getElementById('keyboard-name-input');
        const name = input.value.trim();

        if (name === "") {
            alert("Digite um nome para o teclado!");
            return;
        }

        // Verifica duplicatas
        const alreadyExists = this.keyboardDataToSave.some(k => k.name === name);
        if (alreadyExists) {
            alert("Este teclado já foi adicionado!");
            return;
        }

        const newKeyboard = {
            name: name,
            keys: []
        };

        this.keyboardDataToSave.push(newKeyboard);

        //////// SALVAR O TECLADO NO BANCO DE DADOS
        ////  -----------> AQUI <------------

        this.updateKeyboardSelect();

        // Limpa o input
        input.value = "";

        const select = document.getElementById('keyboard-select');

        Array.from(select.options).forEach(option => {
            if (option.value === name) {
                select.value = name;   
            }
        });

        document.getElementById('save-position-btn').disabled = false;

        // alert("Novo teclado foi criado! Realize o mapeamento das teclas na seção \"Novas Teclas\".");
    }

    updateKeyboardSelect(opt = 1) {

        if (opt) {
            const select = document.getElementById('keyboard-select');

            // Limpa tudo e adiciona a primeira opção
            select.innerHTML = `<option value="">Nenhum teclado selecionado</option>`;

            // Adiciona opções baseado no array
            this.keyboardDataToSave.forEach((keyboard, index) => {
                const option = document.createElement("option");
                option.value = keyboard.name; 
                option.textContent = keyboard.name;
                select.appendChild(option);
            });

            this.renderKeyboardKeys();
        } else {
            const select = document.getElementById('keyboard-select-sec');

            // Limpa tudo e adiciona a primeira opção
            select.innerHTML = `<option value="">Nenhum teclado selecionado</option>`;

            // Adiciona opções baseado no array
            this.keyboardDataToSave.forEach((keyboard, index) => {
                const option = document.createElement("option");
                option.value = keyboard.name; 
                option.textContent = keyboard.name;
                select.appendChild(option);
            });

            this.renderKeyboardKeys(0);
        }
    }

    removeKeys() {
        document.getElementById('keyboard-keys-container').addEventListener('click', (e) => {
            if (e.target.classList.contains('remove-key-btn')) {
                const keyDiv = e.target.closest('.keyboard-key');
                if (keyDiv) {
                    const keyVal = keyDiv.dataset.keyVal;

                    const keyboardId = document.getElementById('keyboard-select').value;
                    const keyboardObj = this.keyboardDataToSave.find(k => k.name == keyboardId);

                    if (keyboardObj) {
                        keyboardObj.keys = keyboardObj.keys.filter(k => k.character !== keyVal);
                    }

                    this.renderKeyboardKeys();
                }
            }
        }); 
    }

    async saveCurrentPosition() {
        const labelInput = document.getElementById('position-label-input');
        const label = labelInput.value.trim().toLowerCase();

        if (!label) {
            this.addLogEntry('Adicione o caractere', 'error');
            return;
        }

        if (label.length !== 1) {
            this.addLogEntry('Deve ser um caractere, não uma palavra', 'error');
            return;
        }

        if (!/[a-z0-9]/.test(label)) {
            this.addLogEntry('Deve ser uma letra ou número', 'error');
            return;
        }

        const x_pos = this.motorPositions[0];
        const y_pos = this.motorPositions[1];

        const keyboardName = document.getElementById('keyboard-select').value;

        const keyboardObj = this.keyboardDataToSave.find(
            element => element.name === keyboardName
        );

        if (!keyboardObj) {
            this.addLogEntry('Teclado não encontrado', 'error');
            return;
        }

        if (!Array.isArray(keyboardObj.keys)) {
            keyboardObj.keys = [];
        }
        
        const existingKey = keyboardObj.keys.find(k => k.character === label);

        if (existingKey) {
            this.addLogEntry(`O caractere '${label}' já foi salvo neste teclado!`, 'error');
            alert(`O caractere '${label}' já foi salvo neste teclado!`);
            return;
        }

        keyboardObj.keys.push({
            character: label,
            xPos: x_pos,
            yPos: y_pos
        });

        this.addLogEntry(`Caractere '${label}' salvo em (${x_pos}, ${y_pos})`, 'success');
        // await this.sendCommand(`SAVEPOS,${label},${x_pos},${y_pos}`);

        this.renderKeyboardKeys();

        labelInput.value = '';
    }

    async recallPosition() {
        const labelInput = document.getElementById('position-label-input');
        const label = labelInput.value.trim().toLowerCase();

        if (!label) {
            this.addLogEntry('Please enter a position label to recall', 'error');
            return;
        }

        if (label.length !== 1) {
            this.addLogEntry('Position label must be a single character', 'error');
            return;
        }

        await this.sendCommand(`RECALLPOS,${label}`);

        // Clear the input after recall
        labelInput.value = '';
    }

    async clearAllPositions() {
        if (confirm('Are you sure you want to clear all saved positions? This cannot be undone.')) {
            await this.sendCommand('CLEARPOS');
        }
    }

    async testAllPositions() {
        if (confirm('Você quer testar todas as posições salvas?')) {
            await this.sendCommand('TESTALLPOS');
        }
    }

    async sendStringToTest() {
        let text = document.querySelector('#string-input').value;

        if (confirm(`Você quer testar a reprodução da palavra: ${text}?`)) {
            await this.sendCommand(`RECALLSTRING,${text}`);
        }
    }

    updateSavedPositionsDisplay() {
        const container = document.getElementById('saved-positions-container');

        if (this.savedPositions.length === 0) {
            container.innerHTML = '<div class="log-entry" style="color: #999;">No positions saved yet...</div>';
            return;
        }

        let html = '';
        for (const pos of this.savedPositions) {
            const colorClass = pos.character.match(/[0-9]/) ? 'info' : '';
            html += `<div class="log-entry ${colorClass}">` +
                    `'${pos.character}' → X:${pos.x_position}, Y:${pos.y_position}</div>`;
        }

        container.innerHTML = html;
        container.scrollTop = container.scrollHeight;
    }

    // controle da tela de log
    addLogEntry(message, type = '') {
        const logContainer = document.getElementById('log-container');
        const entry = document.createElement('div');
        entry.className = `log-entry ${type}`;

        const timestamp = new Date().toLocaleTimeString();
        entry.textContent = `[${timestamp}] ${message}`;

        logContainer.appendChild(entry);
        logContainer.scrollTop = logContainer.scrollHeight;

        while (logContainer.children.length > 100) {
            logContainer.removeChild(logContainer.firstChild);
        }
    }

    clearLog() {
        document.getElementById('log-container').innerHTML = '';
        this.addLogEntry('Área de Log foi limpa', 'info');
    }
}

document.addEventListener('DOMContentLoaded', () => {
    new ControlGUI();
});