using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Threading;

namespace Program
{

     class Program
    {
        // variables for LEX Communication
        private const int Channel = 1;
        private const byte command = 5;
        private const byte Address = 250;
        private SerialPortCommunication _com;
        

        //variables for Display Communication
        private static SerialPort _serialPort;

        //Variable for current Max
        private double currentMax = 0;
        private int displayCounter = 0;
        private bool reset = false;


        static void Main(string[] args)
        {
            Program p = new Program();

             //Setup Display Communication
            p.initDisplay();

            //Setup Lex Communication
            p.SetComPort("/dev/ttyUSB1"); //("/dev/ttyUSB1");
            p.InitCom();

            //reset Max Value of LEX
            p.resetMax();

            //start infinity loop
            p.loop();

        }


        private void loop()
        {
            while (true)
            {
                if (reset == true)
                {
                    resetMax();
                    reset = false;
                    char _startsymbol = (char)2;
                    char _endsymbol = (char)3;
                    string message = _startsymbol + "----" + _endsymbol;
                    //_serialPort.WriteLine(message);
                } else {
                    double value = getMax();
                    Console.WriteLine(value);
                    displayMessage(value);
                }
            }
        }


        private void SetComPort(string portname)
        {
            //FoundComPorts = new ObservableCollection<string>(System.IO.Ports.SerialPort.GetPortNames());
            var port = new System.IO.Ports.SerialPort(portname, 9600, System.IO.Ports.Parity.None, 8, System.IO.Ports.StopBits.One)
            {
                DtrEnable = true,
                RtsEnable = true,
                ReadTimeout = 2000,
                WriteTimeout = 2000
            };
            
            _com = new SerialPortCommunication(port);
            Console.WriteLine("Set Com Port");
            //Console.WriteLine($"{DateTime.Now}: Found Ports: {string.Join(" - ", FoundComPorts)}{Environment.NewLine}");
            
            //KellerProtocol.WakeUp(_com); //remove?
        }

        private void InitCom()
        {
            try
            {
                _com.Open(this);
                KellerProtocol.F48(_com, (byte)Address);
                _com.Close(this);
                Console.WriteLine("Init Communication Protocol (F48)");
                Console.WriteLine("Init Com success");
            }
            catch (Exception exception)
            {
                Console.WriteLine("Init Com Error" + exception);
            }
        }

        private void initDisplay()
        {
            _serialPort = new System.IO.Ports.SerialPort("/dev/ttyUSB0", 9600, System.IO.Ports.Parity.None, 8, System.IO.Ports.StopBits.Two)
            {
                DtrEnable = true,
                RtsEnable = true,
                ReadTimeout = 200,
                WriteTimeout = 200
            };
            _serialPort.Handshake = Handshake.None;
            _serialPort.Open();
            char _startsymbol = (char)2;
            char _endsymbol = (char)3;
            string message = _startsymbol + "LOAD" + _endsymbol;
            _serialPort.WriteLine(message);
            //_serialPort.Close();
        }

        private double getValue()
        {
            try
            {
                _com.Open(this);
                double value = KellerProtocol.F73(_com, (byte)Address, 1);
                _com.Close(this);
                //Console.WriteLine("getValue success");
                return value;
            }
            catch (Exception exception)
            {
                Console.WriteLine("getValue error " + exception);
                return 0;
            }
        }

        private double getMax()
        {
            try
            {
                _com.Open(this);
                double value = KellerProtocol.F73(_com, (byte)Address, 7);
                _com.Close(this);
                return value;
            }
            catch (Exception exception)
            {
                Console.WriteLine("getMax error" + exception);
                return 0;
            }
        }

        private void resetMax()
        {
            //_com.Close(this);
            _com.Open(this);
            KellerProtocol.F95(_com, (byte)Address, command);
            //_com.Close(this);
            Console.WriteLine("resetMax");
            currentMax = 0;
        }



        private void displayMessage(double value)
        {
            if (!_serialPort.IsOpen)
            {
                _serialPort.Open();
            }



            if (value > currentMax)
            {
                currentMax = value;
                displayCounter = 0;

            }
            if (displayCounter >= 10)
            {
                //currentMax = getValue();
                displayCounter = 0;
                reset = true;
            } else
            {
                displayCounter++;
            }


            //int value2 = .ToString();
            string message2 = Math.Round(currentMax * 100).ToString();  //TODO: An Messung später anpassen
            
            if (value <= 1){
                message2 = "O" + message2;
            }
            if (value <= 0.1) {
                message2 = "O" + message2;
            }
            

            char _startsymbol = (char)2;
            char _endsymbol = (char)3;

            string message = _startsymbol + message2 + _endsymbol;
            Console.WriteLine(message2);
            _serialPort.WriteLine(message);
        }


    }



    ////////////////////////////////////////////////////////
    ////////////////Keller Protocol Library/////////////////
    ////////////////////////////////////////////////////////



    /// <summary>
    /// Please refer to document "Description of the Communication protocol" for Series 30 and Series40 pressure transmitters from KELLER
    /// Version 3.3
    /// http://www.keller-druck2.ch/swupdate/BusProtocols/BusProtocols.zip
    /// </summary>
    public static class KellerProtocol
    {
        
        /// <summary>
        /// Write Signal on COM port to wakeup device
        /// </summary>
        /// <returns></returns>
        public static bool WakeUp(ICommunication com)
        {
            try
            {
                F48(com, 250);
                return true;
            }
            catch (CrcException)
            {
                return true;
            }
            catch (TimeoutException)
            {
                return false;
            }
        }

        /// <summary>
        /// Read configuration
        /// </summary>
        /// <param name="com">Communication interface</param>
        /// <param name="address">Device address</param>
        /// <param name="coeffNo">Index to read</param>
        /// <returns>Double</returns>
        /// <exception cref="InvalidDeviceOperationException"></exception>
        /// <exception cref="CrcException"></exception>
        public static double F30(ICommunication com, byte address, byte coeffNo)
        {
            var sndBuffer = new byte[3];
            sndBuffer[0] = address;
            sndBuffer[1] = 30;
            sndBuffer[2] = coeffNo;

            byte[] res = SecureSendReceive(com, sndBuffer, 4);
            Array.Reverse(res);
            return Math.Round(BitConverter.ToSingle(res, 0), 7);
        }


        /// <summary>
        /// Write coefficient
        /// </summary>
        /// <param name="com">Communication interface</param>
        /// <param name="address">Device address</param>
        /// <param name="coeffNo">Index to write</param>
        /// <param name="value">Value to write</param>
        /// <exception cref="InvalidDeviceOperationException"></exception>
        /// <exception cref="CrcException"></exception>
        public static void F31(ICommunication com, byte address, byte coeffNo, float value)
        {
            byte[] b = BitConverter.GetBytes(value);

            var sndBuffer = new byte[7];
            sndBuffer[0] = address;
            sndBuffer[1] = 31;
            sndBuffer[2] = coeffNo;
            sndBuffer[3] = b[3];
            sndBuffer[4] = b[2];
            sndBuffer[5] = b[1];
            sndBuffer[6] = b[0];

            SecureSendReceive(com, sndBuffer, 1);
        }

        /// <summary>
        /// Read Configuration
        /// </summary>
        /// <param name="com">Communication interface</param>
        /// <param name="address">Device address</param>
        /// <param name="coeffNo">Index to read</param>
        /// <returns>Byte</returns>
        /// <exception cref="InvalidDeviceOperationException"></exception>
        /// <exception cref="CrcException"></exception>
        public static byte F32(ICommunication com, byte address, byte coeffNo)
        {
            var sndBuffer = new byte[3];
            sndBuffer[0] = address;
            sndBuffer[1] = 32;
            sndBuffer[2] = coeffNo;

            return SecureSendReceive(com, sndBuffer, 1)[0];
        }

        /// <summary>
        /// Write configuration
        /// </summary>
        /// <param name="com">Communication interface</param>
        /// <param name="address">Device address</param>
        /// <param name="coeffNo">Index to write in</param>
        /// <param name="value">Value to write</param>
        /// <exception cref="InvalidDeviceOperationException"></exception>
        /// <exception cref="CrcException"></exception>
        public static void F33(ICommunication com, byte address, byte coeffNo, byte value)
        {
            var sndBuffer = new byte[4];
            sndBuffer[0] = address;
            sndBuffer[1] = 33;
            sndBuffer[2] = coeffNo;
            sndBuffer[3] = value;

            SecureSendReceive(com, sndBuffer, 1);
        }


        /// <summary>
        /// Initialize and unlock
        /// </summary>
        /// <returns>Byte-Array mit [class|group|year|week]</returns>
        /// <param name="com">Communication interface</param>
        /// <param name="address">Device address</param>
        /// <exception cref="InvalidDeviceOperationException"></exception>
        /// <exception cref="CrcException"></exception>
        public static byte[] F48(ICommunication com, byte address)
        {
            var sndBuffer = new byte[2];
            sndBuffer[0] = address;
            sndBuffer[1] = 48;

            return SecureSendReceive(com, sndBuffer, 6);
        }


        /// <summary>
        /// Write new device address
        /// </summary>
        /// <param name="com">Communication interface</param>
        /// <param name="address">Device address</param>
        /// <param name="newAddress">New address</param>
        /// <returns></returns>
        /// <exception cref="InvalidDeviceOperationException"></exception>
        /// <exception cref="CrcException"></exception>
        public static byte F66(ICommunication com, byte address, byte newAddress)
        {
            var sndBuffer = new byte[3];
            sndBuffer[0] = address;
            sndBuffer[1] = 66;
            sndBuffer[2] = newAddress;

            byte[] res = SecureSendReceive(com, sndBuffer, 1);
            return res[0];
        }


        /// <summary>
        /// Read serial number
        /// </summary>
        /// <param name="com">Communication interface</param>
        /// <param name="address">Device address</param>
        /// <returns>Serial number</returns>
        /// <exception cref="InvalidDeviceOperationException"></exception>
        /// <exception cref="CrcException"></exception>
        public static long F69(ICommunication com, byte address)
        {
            var sndBuffer = new byte[2];
            sndBuffer[0] = address;
            sndBuffer[1] = 69;

            byte[] res = SecureSendReceive(com, sndBuffer, 4);
            Array.Reverse(res);
            return BitConverter.ToInt32(res, 0);
        }

        /// <summary>
        /// Read value of selected channel
        /// </summary>
        /// <param name="com">Communication interface</param>
        /// <param name="address">Device address</param>
        /// <param name="channel">Channel number</param>
        /// <returns></returns>
        /// <exception cref="InvalidDeviceOperationException"></exception>
        /// <exception cref="CrcException"></exception>
        public static double F73(ICommunication com, byte address, byte channel)
        {
            var sndBuffer = new byte[3];
            sndBuffer[0] = address;
            sndBuffer[1] = 73;
            sndBuffer[2] = channel;

            byte[] res = SecureSendReceive(com, sndBuffer, 5);
            Array.Reverse(res);

            // read out state from first byte
            var status = new BitArray(new[] { res[0] });

            if ((channel > status.Length - 1) || (!status[channel] && !status[7]))
            {
                return Math.Round(BitConverter.ToSingle(res, 1), 7);
            }

            return double.NaN;
        }

        /// <summary>
        /// Calculations 
        /// </summary>
        /// <param name="com">Communication interface</param>
        /// <param name="address">Device address</param>
        /// <param name="command">Which channel to set to 0</param>
        /// <returns></returns>
        /// <exception cref="InvalidDeviceOperationException"></exception>
        /// <exception cref="CrcException"></exception>
        public static byte[] F95(ICommunication com, byte address, byte command)
        {
            var sndBuffer = new byte[3];
            sndBuffer[0] = address;
            sndBuffer[1] = 95;
            sndBuffer[2] = command;

            byte[] result = SecureSendReceive(com, sndBuffer, 1);
            return result;
        }

        /// <summary>
        /// Calculations
        /// </summary>
        /// <param name="com">Communication interface</param>
        /// <param name="address">Device address</param>
        /// <param name="command">Which channel to set to certain value</param>
        /// <param name="value">..the certain value</param>
        /// <returns></returns>
        /// <exception cref="InvalidDeviceOperationException"></exception>
        /// <exception cref="CrcException"></exception>
        public static byte[] F95(ICommunication com, byte address, byte command, double value)
        {
            byte[] b = BitConverter.GetBytes((float)value);

            var sndBuffer = new byte[7];
            sndBuffer[0] = address;
            sndBuffer[1] = 95;
            sndBuffer[2] = command;
            sndBuffer[3] = b[3];
            sndBuffer[4] = b[2];
            sndBuffer[5] = b[1];
            sndBuffer[6] = b[0];

            byte[] result = SecureSendReceive(com, sndBuffer, 1);
            return result;
        }

        /// <summary>
        /// Read Configuration
        /// </summary>
        /// <param name="com">Communication interface</param>
        /// <param name="address">Device address</param>
        /// <param name="bteIndex">Index</param>
        /// <returns></returns>
        /// <exception cref="InvalidDeviceOperationException"></exception>
        /// <exception cref="CrcException"></exception>
        public static byte[] F100(ICommunication com, byte address, byte bteIndex)
        {
            var sndBuffer = new byte[3];
            sndBuffer[0] = address;
            sndBuffer[1] = 100;
            sndBuffer[2] = bteIndex;

            return SecureSendReceive(com, sndBuffer, 5);
        }

        /// <summary>
        /// Write Configuration
        /// </summary>
        /// <param name="com">Communication interface</param>
        /// <param name="address">Device address</param>
        /// <param name="index">Index</param>
        /// <param name="values">new values</param>
        /// <exception cref="InvalidDeviceOperationException"></exception>
        /// <exception cref="CrcException"></exception>
        public static void F101(ICommunication com, byte address, byte index, byte[] values)
        {
            var sndBuffer = new byte[8];
            sndBuffer[0] = address;
            sndBuffer[1] = 101;
            sndBuffer[2] = index;
            sndBuffer[3] = values[0];
            sndBuffer[4] = values[1];
            sndBuffer[5] = values[2];
            sndBuffer[6] = values[3];
            sndBuffer[7] = values[4];

            SecureSendReceive(com, sndBuffer, 1);
        }

        private static byte[] SecureSendReceive(ICommunication com, byte[] sndBuffer, int expectedReceiveBytes)
        {
            try
            {
                return SendReceive(com, sndBuffer, expectedReceiveBytes);
            }
            catch (DeviceNotInitializedException)
            {
                if (TryWakeupToRetryCommand(com))
                {
                    return SendReceive(com, sndBuffer, expectedReceiveBytes);
                }

                throw;
            }
            catch (TimeoutException)
            {
                if (TryWakeupToRetryCommand(com))
                {
                    return SendReceive(com, sndBuffer, expectedReceiveBytes);
                }

                throw;
            }
        }

        private static bool TryWakeupToRetryCommand(ICommunication com)
        {

            var sndBuffer = new byte[2];
            sndBuffer[0] = 250;
            sndBuffer[1] = 48;
            try
            {
                SendReceive(com, sndBuffer, 6);
                return true;
            }
            catch (CrcException)
            {
                return true;
            }
            catch (TimeoutException)
            {
                return false;
            }
        }

        private static byte[] SendReceive(ICommunication com, byte[] sndBuffer, int expectedReceiveBytes)
        {
            byte[] rcfBuffer = null;
            var offset = 0;
            int expectedBytes = expectedReceiveBytes + 4;

            // add CRC
            var toSend = new byte[sndBuffer.Length + 2];
            byte[] crc = Crc16(sndBuffer, 0, sndBuffer.Length);

            for (var i = 0; i < sndBuffer.Length; i++)
                toSend[i] = sndBuffer[i];

            toSend[toSend.Length - 2] = crc[0];
            toSend[toSend.Length - 1] = crc[1];

            if (com.EchoOn)
            {
                offset = toSend.Length;
                expectedBytes = expectedReceiveBytes + 4 + toSend.Length;
            }

            try
            {
                com.Send(toSend, out rcfBuffer, expectedBytes);
            }
            catch (TimeoutException e)
            {
                // check for device error
                if (rcfBuffer != null && rcfBuffer[offset + 1] < 127)
                {
                    throw new TimeoutException(e.Message);
                }

                expectedBytes = offset + 5;
            }

            if (rcfBuffer == null)
            {
                throw new Exception();
            }

            // check CRC
            crc = Crc16(rcfBuffer, offset, expectedBytes - 2 - offset);

            if ((crc[0] != rcfBuffer[expectedBytes - 2]) || (crc[1] != rcfBuffer[expectedBytes - 1]))
            {
                throw new CrcException("F" + sndBuffer[1] + " send:" + string.Join(", ", toSend) + "   receive:" +
                                       string.Join(",", rcfBuffer) + "  calc-crc:" + string.Join(",", crc));
            }

            // Device-Exceptions
            if (rcfBuffer[offset + 1] > 127)
            {
                string ret = string.Join(",", rcfBuffer);

                switch (rcfBuffer[offset + 2])
                {
                    case 1:
                        {
                            throw new NotImplementedFunctionException(com.Name + " receive:" + ret);
                        }
                    case 2:
                        {
                            throw new InvalidDeviceOperationException(com.Name + " receive:" + ret);
                        }
                    case 3:
                        {
                            throw new MessageLengthException(com.Name + " receive:" + ret);
                        }
                    case 32:
                        {
                            throw new DeviceNotInitializedException(com.Name + " receive:" + ret);
                        }
                }
            }

            // Check sender
            if (rcfBuffer[offset] != sndBuffer[0])
                throw new Exception();


            // Prepare result
            var result = new byte[expectedReceiveBytes];
            for (int i = offset + 2; i < rcfBuffer.Length - 2; i++)
                result[i - offset - 2] = rcfBuffer[i];

            return result;
        }


        private static byte[] Crc16(byte[] buffer, int offset, int bteCount)
        {
            const ushort polynom = 0xA001;

            ushort crc = 0xFFFF;

            for (var i = 0; i < bteCount; i++)
            {
                crc = (ushort)(crc ^ buffer[offset + i]);

                for (var n = 0; n < 8; n++)
                {
                    bool ex = crc % 2 == 1;
                    crc = (ushort)(crc / 2);
                    if (ex)
                        crc = (ushort)(crc ^ polynom);
                }
            }

            return new[] { (byte)(crc >> 8), (byte)(crc & 0x00ff) };
        }
    }

    public class SerialPortCommunication : ICommunication
    {
        private SerialPort _serialPort;
        private bool _saveMode;
        private string _comName;
        private readonly Dictionary<string, object> _config;

        private readonly object _lockThis;

        /// <summary>default baudrates</summary>
        public static int[] DefaultBaudrates =
        {
            110, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400,
            460800, 921600
        };

        /// <inheritdoc />
        public SerialPortCommunication(SerialPort serialPort)
            : this(serialPort, true)
        {
        }

        /// <param name="serialPort">COM Port</param>
        /// <param name="autoEcho">If true -> echo will recognized automatically</param>
        private SerialPortCommunication(SerialPort serialPort, bool autoEcho)
        {
            _serialPort = serialPort;
            this.EchoOn = true;
            _comName = serialPort.PortName;
            _lockThis = _comName; // Lock object should be a object that is unique to the serial port.

            _saveMode = (serialPort.BaudRate == 9600);

            _config = new Dictionary<string, object>
            {
                {"autoecho", autoEcho},
                {"baudrate", serialPort.BaudRate},
                {"parity", serialPort.Parity},
                {"stopbits", serialPort.StopBits},
                {"readtimeout", serialPort.ReadTimeout},
                {"writetimeout", serialPort.WriteTimeout},
                {"handshake", serialPort.Handshake},
                {"databits", serialPort.DataBits}
            };
        }

        /// <inheritdoc />
        public string Name => (_serialPort == null) ? _comName : _serialPort.PortName;

        /// <summary>
        /// = true -> The COM port gives back an Echo
        /// </summary>
        public bool EchoOn { get; set; }

        /// <summary>
        /// =true -> Echo will be recognized automatically during opening of COM port 
        /// </summary>
        public bool AutoEcho
        {
            get => (bool)GetConfig("autoecho");
            set => SetConfig("autoecho", value);
        }

        /// <inheritdoc />
        public bool IsOpen => (_serialPort != null && _serialPort.IsOpen);

        /// <summary>
        /// Baudrate from COM Port (most likely 9600)
        /// </summary>
        public int Speed
        {
            get => _serialPort.BaudRate;
            set
            {
                if (_serialPort.BaudRate != value) _serialPort.BaudRate = value;
            }
        }


        /// <summary>
        /// Stop-Bit configuration
        /// </summary>
        public StopBits StopBits => _serialPort.StopBits;

        /// <summary>
        /// Parity-Bit configuration
        /// </summary>
        public Parity ParityBits => _serialPort.Parity;


        /// <inheritdoc />
        public object Interface
        {
            get => _serialPort;
            set
            {
                if (value is SerialPort port)
                {
                    lock (_lockThis)
                    {
                        _serialPort = port;
                        _comName = _serialPort.PortName;
                        if (!_serialPort.IsOpen)
                            OpenPort();
                    }
                }
                else if (value == null)
                {
                    _serialPort = null;
                }
            }
        }


        /// <inheritdoc />
        public void SetConfig(Dictionary<string, object> newConfig)
        {
            foreach (KeyValuePair<string, object> kv in newConfig)
            {
                SetConfig(kv.Key, kv.Value);
            }
        }


        /// <inheritdoc />
        public bool SetConfig(string key, object value)
        {
            if (_serialPort == null) return false;

            bool changed = false;

            switch (key)
            {
                case "handshake":
                    {
                        Handshake hs = (Handshake)value;
                        if (hs != _serialPort.Handshake)
                        {
                            _serialPort.Handshake = hs;
                            changed = true;
                            _config["handshake"] = hs;
                        }
                        break;
                    }
                case "baudrate":
                    {
                        int bd = (int)value;
                        if (bd != _serialPort.BaudRate)
                        {
                            _serialPort.BaudRate = bd;
                            changed = true;
                            _config["baudrate"] = bd;
                        }
                        if (bd == 9600)
                            _saveMode = true;
                        break;
                    }
                case "parity":
                    {
                        Parity parity = (Parity)value;
                        if (parity != _serialPort.Parity)
                        {
                            _serialPort.Parity = parity;
                            _config["parity"] = parity;
                            changed = true;
                        }
                        break;
                    }
                case "stopbits":
                    {
                        StopBits stop = (StopBits)value;
                        if (stop != _serialPort.StopBits)
                        {
                            _serialPort.StopBits = stop;
                            _config["stopbits"] = stop;
                            changed = true;
                        }
                        break;
                    }
                case "readtimeout":
                    {
                        int timeout = (int)value;
                        if (timeout != _serialPort.ReadTimeout)
                        {
                            _serialPort.ReadTimeout = timeout;
                            _config["readtimeout"] = timeout;
                            changed = true;
                        }
                        break;
                    }
                case "writetimeout":
                    {
                        int timeout = (int)value;
                        if (timeout != _serialPort.WriteTimeout)
                        {
                            _serialPort.WriteTimeout = timeout;
                            _config["writetimeout"] = timeout;
                            changed = true;
                        }
                        break;
                    }
                case "databits":
                    {
                        int databits = (int)value;
                        if (databits != _serialPort.DataBits)
                        {
                            _serialPort.DataBits = databits;
                            _config["databits"] = databits;
                            changed = true;
                        }
                        break;
                    }
                case "autoecho":
                    {
                        bool autoecho = (bool)value;
                        if (autoecho != (bool)_config["autoecho"])
                        {
                            _config["autoecho"] = autoecho;
                            changed = true;
                        }
                        break;
                    }
            }

            return changed;
        }

        /// <inheritdoc />
        public object GetConfig(string key)
        {
            return _config.ContainsKey(key) ? _config[key] : null;
        }

        /// <inheritdoc />
        public Dictionary<string, object> GetConfigCopy()
        {
            var cfg = new Dictionary<string, object>();
            foreach (KeyValuePair<string, object> kv in _config)
                cfg.Add(kv.Key, kv.Value);

            return cfg;
        }

        /// <inheritdoc />
        public void Send(byte[] command, out byte[] rcfBuffer, int readByteCount)
        {
            rcfBuffer = new byte[readByteCount];
            if (_serialPort == null) return;

            lock (_lockThis)
            {
                try
                {
                    //    if (!_serialPort.IsOpen) return;
                    // clear received buffer
                    _serialPort.ReadExisting();

                    // write command data
                    if (_saveMode)
                        Thread.Sleep(1);
                    _serialPort.Write(command, 0, command.Length);


                    // receive data
                    int a = 0;
                    while (a < readByteCount)
                    {
                        rcfBuffer[a] = (byte)_serialPort.ReadByte();
                        a++;
                    }
                }
                catch (InvalidOperationException)
                {
                }
                catch (UnauthorizedAccessException)
                {
                }
            }
        }

        /// <inheritdoc />
        public void Send(byte[] command, out byte[] rcfBuffer, byte endSign)
        {
            rcfBuffer = new byte[0];

            if (_serialPort == null) return;

            lock (_lockThis)
            {
                try
                {
                    // clear received buffer
                    _serialPort.ReadExisting();

                    // write command data
                    if (_saveMode)
                        Thread.Sleep(1);
                    _serialPort.Write(command, 0, command.Length);

                    // receive data
                    do
                    {
                        Array.Resize(ref rcfBuffer, rcfBuffer.Length + 1);
                        rcfBuffer[rcfBuffer.Length - 1] = (byte)_serialPort.ReadByte();
                    } while (rcfBuffer[rcfBuffer.Length - 1] != endSign);
                }
                catch (UnauthorizedAccessException)
                {
                }
            }
        }

        /// <summary>
        /// Opens COM port. The COM port will be opened "virtually" for each object
        /// and closed again when the last port is closed.
        /// </summary>
        /// <param name="sender">Object that wants to open the COM port. Used mostly with "this"</param>
        public void Open(object sender)
        {
            if (_serialPort == null) return;

            lock (_lockThis)
            {
                if (!_serialPort.IsOpen)
                    OpenPort();
            }
        }

        private void OpenPort()
        {
            if (_serialPort == null) return;

            try
            {
                //Debug.WriteLine("?Try to open Port: (before) " + _serialPort.PortName);
                _serialPort.Open();
                //Debug.WriteLine("?Try to open Port: (wait) " + _serialPort.PortName);
                Thread.Sleep(100);
                //Debug.WriteLine("?Try to open Port: (after wait) " + _serialPort.PortName);
                _serialPort.DiscardInBuffer();
                if ((bool)GetConfig("autoecho"))
                    CheckEcho();
            }
            catch (UnauthorizedAccessException)
            {
                //Debug.WriteLine("Comport OpenPort():" + e.GetType().ToString() + " -> " + e.Message);
            }
            catch (IOException)
            {
                //http://stackoverflow.com/questions/14885288/io-exception-error-when-using-serialport-open
                //Debug.WriteLine(ioexp);
            }
        }


        /// <summary>
        /// Close the Comport
        /// </summary>
        /// <param name="sender">Object representing the COM port to be closed</param>
        public void Close(object sender)
        {
            //     _useComPort.Remove(sender);
            if ((_serialPort == null) || (!_serialPort.IsOpen)) return;

            lock (_lockThis)
            {
                try
                {
                    Debug.WriteLine("?Try to close Port: (before) " + _serialPort.PortName);
                    _serialPort.Close();
                    Debug.WriteLine("?Try to close Port: (wait) " + _serialPort.PortName);
                    Thread.Sleep(500);
                }
                catch (UnauthorizedAccessException)
                {
                }
            }
        }

        private void CheckEcho()
        {
            if (_serialPort == null) return;

            lock (_lockThis)
            {
                _serialPort.Write("e");
                Thread.Sleep(250);
                EchoOn = (_serialPort.ReadExisting() == "e");
            }
        }


        /// <summary>
        /// Outputs the COM port name eg. "COM1"
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return Name;
        }
    }

    /// <summary>
    /// Interface for a communication interface such as System.IO.SerialPort, Windows.Devices.SerialCommunication, TCP/IP or Bluetooth
    /// </summary>
    public interface ICommunication
    {
        /// <summary>Name of the interface (eg. SerialPort => "COM12")</summary>
        string Name { get; }

        /// <summary>= true, if the interface gives back an Echo</summary>
        bool EchoOn { get; set; }

        /// <summary>= true, Echo will be recognized automatically</summary>
        bool AutoEcho { get; set; }

        /// <summary>= true, when interface is open</summary>
        bool IsOpen { get; }

        /// <summary>speed of interface (With serial ports it is measured in Baudrate)</summary>
        int Speed { get; }

        /// <summary>The interface</summary>
        object Interface { get; set; }

        /// <summary>
        /// Send AND receive data from interface
        /// </summary>
        /// <param name="command">send data</param>
        /// <param name="rcfBuffer">received data</param>
        /// <param name="readByteCount">Count of to received bytes</param>
        void Send(byte[] command, out byte[] rcfBuffer, int readByteCount);

        /// <summary>
        /// Send AND receive data from interface
        /// </summary>
        /// <param name="command">send data</param>
        /// <param name="rcfBuffer">received data</param>
        /// <param name="endSign">receive until this character</param>
        void Send(byte[] command, out byte[] rcfBuffer, byte endSign);

        /// <summary>
        /// opens the interface
        /// </summary>
        /// <param name="sender">origin object that wants to open</param>
        void Open(object sender);

        /// <summary>
        /// closes the interface
        /// </summary>
        /// <param name="sender">origin object that wants to open</param>
        void Close(object sender);

        /// <summary>
        /// Configure multiple configure parameters at once
        /// </summary>
        /// <param name="newConfig">New parameters</param>
        void SetConfig(Dictionary<string, object> newConfig);

        /// <summary>
        /// Configures single configure parameter
        /// </summary>
        /// <param name="key">configuration key</param>
        /// <param name="value">configuration value</param>
        /// <returns>=true, if configuration was changed</returns>
        bool SetConfig(string key, object value);

        /// <summary>
        /// Read out one configuration parameter
        /// </summary>
        /// <param name="key">key</param>
        /// <returns>Value</returns>
        object GetConfig(string key);

        /// <summary>
        /// Read out all configuration parameters
        /// </summary>
        /// <returns>all configuration parameters</returns>
        Dictionary<string, object> GetConfigCopy();
    }

    // Exceptions
    public class CrcException : Exception
    {
        public CrcException(string message) : base(message)
        {
        }

        public CrcException()
        {
        }
    }

    public class AnswerException : Exception
    {
        public AnswerException(string message) : base(message)
        {
        }

        public AnswerException()
        {
        }
    }

    public class NotImplementedFunctionException : Exception
    {
        public NotImplementedFunctionException(string message) : base(message)
        {
        }

        public NotImplementedFunctionException()
        {
        }
    }

    public class DeviceNotInitializedException : Exception
    {
        public DeviceNotInitializedException(string message) : base(message)
        {
        }

        public DeviceNotInitializedException()
        {
        }
    }

    public class InvalidDeviceOperationException : Exception
    {
        public InvalidDeviceOperationException(string message) : base(message)
        {
        }

        public InvalidDeviceOperationException()
        {
        }
    }

    public class MessageLengthException : Exception
    {
        public MessageLengthException(string message) : base(message)
        {
        }

        public MessageLengthException()
        {
        }
    }













   


}
