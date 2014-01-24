
var OREmulatorModule = null;  // Global application object.
var emulatorTerminal = null;
var perfValue = null;

function DebugMessage(message) {
  console.log(message);
}

// Small UART device
function UARTDev(worker) {
  this.ReceiveChar = function(c) {
    worker.sendToWorker(0x01000000 | c);
  };
}

// Terminal for emulator UART output
function EmulatorTerminal(canvasID) {
  this.term = new Terminal(40, 120, canvasID);
  this.termInput = new TerminalInput(new UARTDev(this));

  this.termCanvas = document.getElementById(canvasID);

  this.sendToWorker = function(c) {
    if (OREmulatorModule !== null) {
      OREmulatorModule.postMessage(c);
    } else {
      //console.log ('ERROR: failed to send message to emulator as it is not yet initialised');
      //console.log (c);
    }
  }.bind(this);

  document.onkeypress = function(event) {
    this.sendToWorker(0x02000000 | event.keyCode);
    return this.termInput.OnKeyPress(event);      
  }.bind(this);
  
  document.onkeydown = function(event) {
    this.sendToWorker(0x03000000 | event.keyCode);
    return this.termInput.OnKeyDown(event);
  }.bind(this);
  
  document.onkeyup = function(event) {
    this.sendToWorker(0x04000000 | event.keyCode);
    return this.termInput.OnKeyUp(event);
  }.bind(this);
}

function moduleDidLoad() {
  // Indicate module load success
  OREmulatorModule = document.getElementById('or-emulator');
  logToTerminal('Successfully loaded module');

  // Start up the cycle count requester
  setInterval(function () {
    OREmulatorModule.postMessage(0x05000000);
  }, 1000);
}

function handleMessage(message_event) {
  if (typeof message_event.data === 'string') {
    console.log(message_event.data);
  } else if (typeof message_event.data === 'number') {
    msgType = message_event.data >>> 24;
    msg = message_event.data & 0x00ffffff;

    switch (msgType) {
    case 0x00:
      // Character to send to the emulated terminal display
      if (emulatorTerminal !== null) {
        emulatorTerminal.term.PutChar(msg);
      }
      break;
    case 0x01:
      // MIPS value
      perfValue.innerHTML = msg + ' MIPS';
      break;
      
    default:
      break;
    }
    
  } else {
    console.log('ERROR: unexpected message type from OpenRISC emulator');
  }
}

function pageDidLoad() {
  // Setup the terminal
  emulatorTerminal = new EmulatorTerminal('or-terminal');

  // Set the initial module load status
  if (OREmulatorModule == null) {
    logToTerminal('Loading module...');
  }
  
  perfValue = document.getElementById('perf-value');
}

function logToTerminal(msg) {
  if (emulatorTerminal != null) {
    var c;
    for (var i = 0; i < msg.length; ++i) {
      emulatorTerminal.term.PutChar(msg.charCodeAt(i));
    }
    emulatorTerminal.term.PutChar('\r'.charCodeAt(0));
    emulatorTerminal.term.PutChar('\n'.charCodeAt(0));
  }
}

// Startup
var listener = document.getElementById('listener');
listener.addEventListener('load', moduleDidLoad, true);
listener.addEventListener('message', handleMessage, true);
