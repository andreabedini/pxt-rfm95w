/**
 * LoRa radio driver for the HopeRF RFM95W / Semtech SX127x chipset.
 * Lets micro:bit programs send and receive text, numbers and raw bytes over
 * long-range LoRa radio without an internet connection.
 */
//% color=#e67e22 icon="" weight=8
namespace Lora {

    /**
     * Frequency band to use.  Choose the band that matches the radio module
     * you purchased and the regulations in your country.
     */
    export enum Band {
        //% block="EU (868 MHz)"
        EU868 = 868,
        //% block="AU / US (915 MHz)"
        AU915 = 915
    }

    /**
     * LoRa spreading factor.  Higher values give longer range at the cost of
     * slower data rate.  Both ends of a link must use the same spreading factor.
     */
    export enum SpreadingFactor {
        //% block="SF7 (fastest, shortest range)"
        SF7 = 7,
        //% block="SF8"
        SF8 = 8,
        //% block="SF9"
        SF9 = 9,
        //% block="SF10"
        SF10 = 10,
        //% block="SF11"
        SF11 = 11,
        //% block="SF12 (slowest, longest range)"
        SF12 = 12
    }

    /**
     * LoRa signal bandwidth.  Narrower bandwidth gives longer range;
     * wider bandwidth gives faster data rate.  Both ends must match.
     */
    export enum Bandwidth {
        //% block="7.8 kHz"
        BW7k8 = 0,
        //% block="10.4 kHz"
        BW10k4 = 1,
        //% block="15.6 kHz"
        BW15k6 = 2,
        //% block="20.8 kHz"
        BW20k8 = 3,
        //% block="31.25 kHz"
        BW31k25 = 4,
        //% block="41.7 kHz"
        BW41k7 = 5,
        //% block="62.5 kHz"
        BW62k5 = 6,
        //% block="125 kHz (default)"
        BW125k = 7,
        //% block="250 kHz"
        BW250k = 8,
        //% block="500 kHz (fastest, shortest range)"
        BW500k = 9
    }

    /**
     * LoRa forward error-correction coding rate.  Higher rates add more
     * redundancy so packets survive noisier conditions, but reduce throughput.
     * Both ends must match.
     */
    export enum CodingRate {
        //% block="4/5 (default)"
        CR45 = 1,
        //% block="4/6"
        CR46 = 2,
        //% block="4/7"
        CR47 = 3,
        //% block="4/8 (most error correction)"
        CR48 = 4
    }

    /**
     * Property of the last received packet.
     */
    export enum PacketProperty {
        //% block="time"
        Time = 0,
        //% block="serial number"
        SerialNumber = 1,
        //% block="signal strength"
        SignalStrength = 2
    }

    // --- SX127x register map (subset) ---
    const REG_FIFO = 0x00
    const REG_OP_MODE = 0x01
    const REG_FRF_MSB = 0x06
    const REG_FRF_MID = 0x07
    const REG_FRF_LSB = 0x08
    const REG_PA_CONFIG = 0x09
    const REG_LNA = 0x0C
    const REG_FIFO_ADDR_PTR = 0x0D
    const REG_FIFO_TX_BASE_ADDR = 0x0E
    const REG_FIFO_RX_BASE_ADDR = 0x0F
    const REG_FIFO_RX_CURRENT_ADDR = 0x10
    const REG_IRQ_FLAGS = 0x12
    const REG_RX_NB_BYTES = 0x13
    const REG_PKT_SNR_VALUE = 0x19
    const REG_PKT_RSSI_VALUE = 0x1A
    const REG_RSSI_VALUE = 0x1B
    const REG_MODEM_CONFIG_1 = 0x1D
    const REG_MODEM_CONFIG_2 = 0x1E
    const REG_PREAMBLE_MSB = 0x20
    const REG_PREAMBLE_LSB = 0x21
    const REG_PAYLOAD_LENGTH = 0x22
    const REG_MODEM_CONFIG_3 = 0x26
    const REG_SYNC_WORD = 0x39
    const REG_VERSION = 0x42
    const REG_PA_DAC = 0x4D

    // --- Modes / bits ---
    const MODE_LONG_RANGE = 0x80 // LoRa mode bit in RegOpMode
    const MODE_SLEEP = 0x00
    const MODE_STDBY = 0x01
    const MODE_TX = 0x03
    const MODE_RX_CONTINUOUS = 0x05

    // IRQ flags
    const IRQ_RX_DONE = 0x40
    const IRQ_TX_DONE = 0x08
    const IRQ_PAYLOAD_CRC_ERROR = 0x20
    const IRQ_CLEAR_ALL = 0xFF

    // Packet wire format (compatible with micro:bit radio extension):
    // | 0           | 1 ... 4     | 5 ... 8       | 9 ...
    // | packet type | system time | serial number | payload
    const PACKET_PREFIX_LENGTH = 9
    const PACKET_TYPE_NUMBER = 0       // payload: Int32LE
    const PACKET_TYPE_VALUE = 1        // payload: Int32LE, name length, name
    const PACKET_TYPE_STRING = 2       // payload: string length, string
    const PACKET_TYPE_BUFFER = 3       // payload: buffer length, buffer
    const PACKET_TYPE_DOUBLE = 4       // payload: Float64LE
    const PACKET_TYPE_DOUBLE_VALUE = 5 // payload: Float64LE, name length, name
    const VALUE_PACKET_NAME_LEN_OFFSET = 13
    const DOUBLE_VALUE_PACKET_NAME_LEN_OFFSET = 17
    const MAX_NAME_LENGTH = 8

    // BW lookup table: bandwidth in kHz × 100 (integer arithmetic, no floats)
    const BW_KHZ_X100 = [780, 1040, 1560, 2080, 3125, 4170, 6250, 12500, 25000, 50000]

    // Wiring defaults (override if you want later)
    let _nss = DigitalPin.P16
    let _rst = DigitalPin.P8

    // micro:bit default SPI pins are P13/P14/P15
    let _mosi = DigitalPin.P15
    let _miso = DigitalPin.P14
    let _sck = DigitalPin.P13

    let _inited = false
    let _transmitSerial = false
    let _onReceivedNumberHandler: (receivedNumber: number) => void = null
    let _onReceivedValueHandler: (name: string, value: number) => void = null
    let _onReceivedStringHandler: (receivedString: string) => void = null
    let _onReceivedBufferHandler: (receivedBuffer: Buffer) => void = null
    let _lastRssi = 0
    let _lastSnrRaw = 0        // raw REG_PKT_SNR_VALUE (uint8 representing signed byte)
    let _lastPacketTime = 0
    let _lastPacketSerial = 0
    let _sf = 7                // current spreading factor (7–12)
    let _bwCode = 7            // current BW register code (default 125 kHz)

    function csSelect() { pins.digitalWritePin(_nss, 0) }
    function csDeselect() { pins.digitalWritePin(_nss, 1) }

    function spiInit() {
        pins.spiPins(_mosi, _miso, _sck)
        pins.spiFrequency(1000000) // start conservative; can go higher later
        pins.spiFormat(8, 0) // 8 bits, mode 0
        pins.digitalWritePin(_nss, 1)
    }

    function readReg(addr: number): number {
        csSelect()
        pins.spiWrite(addr & 0x7F)
        const v = pins.spiWrite(0x00)
        csDeselect()
        return v
    }

    function writeReg(addr: number, value: number): void {
        csSelect()
        pins.spiWrite(addr | 0x80)
        pins.spiWrite(value)
        csDeselect()
    }

    function burstWrite(addr: number, buf: Buffer): void {
        csSelect()
        pins.spiWrite(addr | 0x80)
        for (let i = 0; i < buf.length; i++) pins.spiWrite(buf[i])
        csDeselect()
    }

    function burstRead(addr: number, len: number): Buffer {
        const out = pins.createBuffer(len)
        csSelect()
        pins.spiWrite(addr & 0x7F)
        for (let i = 0; i < len; i++) out[i] = pins.spiWrite(0x00)
        csDeselect()
        return out
    }

    function resetChip() {
        pins.digitalWritePin(_rst, 0)
        basic.pause(2)
        pins.digitalWritePin(_rst, 1)
        basic.pause(10)
    }

    function setMode(mode: number) {
        writeReg(REG_OP_MODE, MODE_LONG_RANGE | mode)
    }

    function setFrequencyMHz(mhz: number) {
        // FRF = (freq_hz * 2^19) / 32e6
        const freqHz = mhz * 1000000
        const frf = Math.floor(freqHz * 524288 / 32000000)
        writeReg(REG_FRF_MSB, (frf >> 16) & 0xFF)
        writeReg(REG_FRF_MID, (frf >> 8) & 0xFF)
        writeReg(REG_FRF_LSB, frf & 0xFF)
    }

    function applyLdro() {
        const needLdro = ((1 << _sf) * 100) > (16 * BW_KHZ_X100[_bwCode])
        let v = readReg(REG_MODEM_CONFIG_3)
        writeReg(REG_MODEM_CONFIG_3, needLdro ? (v | 0x08) : (v & 0xF7))
    }

    function configRadioDefaults(band: Band) {
        // FIFO bases
        writeReg(REG_FIFO_TX_BASE_ADDR, 0x00)
        writeReg(REG_FIFO_RX_BASE_ADDR, 0x00)
        writeReg(REG_FIFO_ADDR_PTR, 0x00)

        // Preamble
        writeReg(REG_PREAMBLE_MSB, 0x00)
        writeReg(REG_PREAMBLE_LSB, 0x08) // 8 symbols

        // Sync word: 0x34 is common for "public LoRa" style networks
        writeReg(REG_SYNC_WORD, 0x34)

        // Frequency
        if (band == Band.EU868) setFrequencyMHz(868.0)
        else setFrequencyMHz(915.0)

        // Modem config: BW=125kHz, CR=4/5, Explicit header
        // RegModemConfig1: BW bits 7..4, CR bits 3..1
        writeReg(REG_MODEM_CONFIG_1, 0x72) // 0b0111_0010
        // RegModemConfig2: SF=7 (0x70), CRC on (bit 2)
        writeReg(REG_MODEM_CONFIG_2, 0x74)
        // RegModemConfig3: LowDataRateOptimize off, AGC auto on
        writeReg(REG_MODEM_CONFIG_3, 0x04)

        // LNA boost
        writeReg(REG_LNA, readReg(REG_LNA) | 0x03)

        // PA_BOOST path; lower 4 bits are output power
        writeReg(REG_PA_CONFIG, 0x80 | 0x0F)
        writeReg(REG_PA_DAC, 0x84) // 0x87 enables 20 dBm
    }

    function transmit(buf: Buffer): void {
        setMode(MODE_STDBY)
        writeReg(REG_IRQ_FLAGS, IRQ_CLEAR_ALL)
        writeReg(REG_FIFO_ADDR_PTR, 0x00)
        burstWrite(REG_FIFO, buf)
        writeReg(REG_PAYLOAD_LENGTH, buf.length)
        setMode(MODE_TX)
        pauseUntil(() => (readReg(REG_IRQ_FLAGS) & IRQ_TX_DONE) != 0)
        writeReg(REG_IRQ_FLAGS, IRQ_TX_DONE)
        setMode(MODE_RX_CONTINUOUS)
    }

    function handlePacket(raw: Buffer): void {
        if (raw.length < PACKET_PREFIX_LENGTH) return
        _lastPacketTime = raw.getNumber(NumberFormat.Int32LE, 1)
        _lastPacketSerial = raw.getNumber(NumberFormat.Int32LE, 5)
        const t = raw[0]
        if (t == PACKET_TYPE_NUMBER && raw.length >= PACKET_PREFIX_LENGTH + 4) {
            if (_onReceivedNumberHandler)
                _onReceivedNumberHandler(raw.getNumber(NumberFormat.Int32LE, PACKET_PREFIX_LENGTH))
        } else if (t == PACKET_TYPE_DOUBLE && raw.length >= PACKET_PREFIX_LENGTH + 8) {
            if (_onReceivedNumberHandler)
                _onReceivedNumberHandler(raw.getNumber(NumberFormat.Float64LE, PACKET_PREFIX_LENGTH))
        } else if (t == PACKET_TYPE_STRING && raw.length > PACKET_PREFIX_LENGTH) {
            if (_onReceivedStringHandler) {
                const len = raw[PACKET_PREFIX_LENGTH]
                _onReceivedStringHandler(raw.slice(PACKET_PREFIX_LENGTH + 1, len).toString())
            }
        } else if (t == PACKET_TYPE_BUFFER && raw.length > PACKET_PREFIX_LENGTH) {
            if (_onReceivedBufferHandler) {
                const len = raw[PACKET_PREFIX_LENGTH]
                _onReceivedBufferHandler(raw.slice(PACKET_PREFIX_LENGTH + 1, len))
            }
        } else if (t == PACKET_TYPE_VALUE && raw.length > VALUE_PACKET_NAME_LEN_OFFSET) {
            if (_onReceivedValueHandler) {
                const num = raw.getNumber(NumberFormat.Int32LE, PACKET_PREFIX_LENGTH)
                const nameLen = raw[VALUE_PACKET_NAME_LEN_OFFSET]
                _onReceivedValueHandler(raw.slice(VALUE_PACKET_NAME_LEN_OFFSET + 1, nameLen).toString(), num)
            }
        } else if (t == PACKET_TYPE_DOUBLE_VALUE && raw.length > DOUBLE_VALUE_PACKET_NAME_LEN_OFFSET) {
            if (_onReceivedValueHandler) {
                const num = raw.getNumber(NumberFormat.Float64LE, PACKET_PREFIX_LENGTH)
                const nameLen = raw[DOUBLE_VALUE_PACKET_NAME_LEN_OFFSET]
                _onReceivedValueHandler(raw.slice(DOUBLE_VALUE_PACKET_NAME_LEN_OFFSET + 1, nameLen).toString(), num)
            }
        }
    }

    /**
     * Initialise the RFM95W radio module and start listening for incoming
     * packets.  Call this once at the start of your program before using any
     * other LoRa blocks.
     * @param band frequency band matching your module and local regulations, eg: Lora.Band.EU868
     */
    //% block="LoRa init band %band"
    //% group="Configuration" weight=100
    export function init(band: Band) {
        spiInit()
        resetChip()

        const version = readReg(REG_VERSION)
        // SX1276/77/78/79 typically reports 0x12.
        if (version != 0x12) {
            _inited = false
            return
        }

        setMode(MODE_SLEEP)
        basic.pause(10)
        setMode(MODE_STDBY)
        basic.pause(10)

        configRadioDefaults(band)

        control.runInBackground(function () {
            while (true) {
                pauseUntil(() => (readReg(REG_IRQ_FLAGS) & IRQ_RX_DONE) != 0)
                const flags = readReg(REG_IRQ_FLAGS)
                if ((flags & IRQ_PAYLOAD_CRC_ERROR) == 0) {
                    const currentAddr = readReg(REG_FIFO_RX_CURRENT_ADDR)
                    writeReg(REG_FIFO_ADDR_PTR, currentAddr)
                    const len = readReg(REG_RX_NB_BYTES)
                    const pkt = burstRead(REG_FIFO, len)
                    _lastRssi = readReg(REG_PKT_RSSI_VALUE) - 157
                    _lastSnrRaw = readReg(REG_PKT_SNR_VALUE)
                    control.runInBackground(function () { handlePacket(pkt) })
                }
                writeReg(REG_IRQ_FLAGS, IRQ_CLEAR_ALL)
            }
        })
        setMode(MODE_RX_CONTINUOUS)

        _inited = true
    }

    /**
     * Send a number over LoRa.  Integers are sent as 32-bit; non-integers as
     * 64-bit double — matching the micro:bit radio packet format.
     * @param value the number to send, eg: 0
     */
    //% block="LoRa send number %value"
    //% group="Send" weight=62
    export function sendNumber(value: number): void {
        if (!_inited) return
        const isInt = value === (value | 0)
        const buf = pins.createBuffer(PACKET_PREFIX_LENGTH + (isInt ? 4 : 8))
        buf[0] = isInt ? PACKET_TYPE_NUMBER : PACKET_TYPE_DOUBLE
        buf.setNumber(NumberFormat.Int32LE, 1, control.millis())
        buf.setNumber(NumberFormat.Int32LE, 5, _transmitSerial ? control.deviceSerialNumber() : 0)
        buf.setNumber(isInt ? NumberFormat.Int32LE : NumberFormat.Float64LE, PACKET_PREFIX_LENGTH, value)
        transmit(buf)
    }

    /**
     * Send a name/value pair over LoRa.  The name can be at most 8 characters.
     * @param name the field name, eg: "temp"
     * @param value the numeric value, eg: 0
     */
    //% block="LoRa send value %name = %value"
    //% group="Send" weight=61
    export function sendValue(name: string, value: number): void {
        if (!_inited) return
        const isInt = value === (value | 0)
        const nameEncoded = control.createBufferFromUTF8(name)
        const nameLen = Math.min(nameEncoded.length, MAX_NAME_LENGTH)
        const numSize = isInt ? 4 : 8
        const buf = pins.createBuffer(PACKET_PREFIX_LENGTH + numSize + 1 + nameLen)
        buf[0] = isInt ? PACKET_TYPE_VALUE : PACKET_TYPE_DOUBLE_VALUE
        buf.setNumber(NumberFormat.Int32LE, 1, control.millis())
        buf.setNumber(NumberFormat.Int32LE, 5, _transmitSerial ? control.deviceSerialNumber() : 0)
        buf.setNumber(isInt ? NumberFormat.Int32LE : NumberFormat.Float64LE, PACKET_PREFIX_LENGTH, value)
        buf[PACKET_PREFIX_LENGTH + numSize] = nameLen
        buf.write(PACKET_PREFIX_LENGTH + numSize + 1, nameEncoded.slice(0, nameLen))
        transmit(buf)
    }

    /**
     * Send a text string over LoRa.  The radio returns to receive mode
     * automatically once transmission is complete.
     * @param value the string to send, eg: "Hello!"
     */
    //% block="LoRa send string %value"
    //% value.shadowOptions.toString=true
    //% group="Send" weight=60
    export function sendString(value: string): void {
        if (!_inited) return
        const encoded = control.createBufferFromUTF8(value)
        const len = encoded.length
        const buf = pins.createBuffer(PACKET_PREFIX_LENGTH + 1 + len)
        buf[0] = PACKET_TYPE_STRING
        buf.setNumber(NumberFormat.Int32LE, 1, control.millis())
        buf.setNumber(NumberFormat.Int32LE, 5, _transmitSerial ? control.deviceSerialNumber() : 0)
        buf[PACKET_PREFIX_LENGTH] = len
        buf.write(PACKET_PREFIX_LENGTH + 1, encoded)
        transmit(buf)
    }

    /**
     * Send a raw buffer of bytes over LoRa.  Use this when you need to send
     * binary data or a custom packet format.
     * @param payload the buffer to transmit
     */
    //% group="Send" weight=59 advanced=true
    export function sendBuffer(payload: Buffer): void {
        if (!_inited) return
        const len = payload.length
        const buf = pins.createBuffer(PACKET_PREFIX_LENGTH + 1 + len)
        buf[0] = PACKET_TYPE_BUFFER
        buf.setNumber(NumberFormat.Int32LE, 1, control.millis())
        buf.setNumber(NumberFormat.Int32LE, 5, _transmitSerial ? control.deviceSerialNumber() : 0)
        buf[PACKET_PREFIX_LENGTH] = len
        buf.write(PACKET_PREFIX_LENGTH + 1, payload)
        transmit(buf)
    }

    /**
     * Run code when a LoRa number is received.
     */
    //% block="on LoRa received $receivedNumber"
    //% draggableParameters="reporter"
    //% group="Receive" weight=52
    export function onReceivedNumber(handler: (receivedNumber: number) => void): void {
        _onReceivedNumberHandler = handler
    }

    /**
     * Run code when a LoRa name/value pair is received.
     */
    //% block="on LoRa received $name = $value"
    //% draggableParameters="reporter"
    //% group="Receive" weight=51
    export function onReceivedValue(handler: (name: string, value: number) => void): void {
        _onReceivedValueHandler = handler
    }

    /**
     * Run code when a LoRa string is received.  The ``receivedString``
     * reporter contains the text that arrived.
     */
    //% block="on LoRa received $receivedString"
    //% draggableParameters="reporter"
    //% group="Receive" weight=50
    export function onReceivedString(handler: (receivedString: string) => void): void {
        _onReceivedStringHandler = handler
    }

    /**
     * Run code when a LoRa buffer is received.
     */
    //% group="Receive" advanced=true weight=49
    export function onReceivedBuffer(handler: (receivedBuffer: Buffer) => void): void {
        _onReceivedBufferHandler = handler
    }

    /**
     * Returns a property of the last received packet.
     * @param type the property to retrieve
     */
    //% block="LoRa received packet %type"
    //% group="Receive" weight=35
    export function receivedPacket(type: PacketProperty): number {
        switch (type) {
            case PacketProperty.Time: return _lastPacketTime
            case PacketProperty.SerialNumber: return _lastPacketSerial
            case PacketProperty.SignalStrength: return _lastRssi
            default: return 0
        }
    }

    /**
     * RSSI of the last received packet in dBm.  Typical values range from
     * about −70 dBm (strong, nearby) to −110 dBm (weak, at the edge of range).
     * Returns 0 if no packet has been received yet.
     */
    //% block="LoRa signal strength (dBm)"
    //% group="Receive" weight=30
    export function receivedSignalStrength(): number {
        return _lastRssi
    }

    /**
     * Signal-to-noise ratio of the last received packet in dB.  Positive
     * values indicate a clean signal; negative values mean the packet arrived
     * below the noise floor (still decodable by LoRa, but marginal).
     * Returns 0 if no packet has been received yet.
     */
    //% block="LoRa received SNR (dB)"
    //% group="Receive" weight=29
    export function receivedSnr(): number {
        const raw = _lastSnrRaw > 127 ? _lastSnrRaw - 256 : _lastSnrRaw
        return raw / 4
    }

    /**
     * Wideband RSSI of the radio channel right now, in dBm.  Unlike
     * ``receivedSignalStrength``, this does not require a packet to have
     * arrived — it reflects the background noise level on the channel and can
     * be used to check whether the channel is busy before transmitting.
     * Returns 0 if the radio has not been initialised.
     */
    //% block="LoRa channel RSSI (dBm)"
    //% group="Receive" weight=28
    export function channelRssi(): number {
        if (!_inited) return 0
        return readReg(REG_RSSI_VALUE) - 157
    }

    /**
     * Set whether to include the device serial number in each sent packet.
     * @param transmit true to include the serial number, eg: true
     */
    //% block="LoRa transmit serial number %transmit"
    //% group="Configuration" advanced=true weight=45
    export function setTransmitSerialNumber(transmit: boolean): void {
        _transmitSerial = transmit
    }

    /**
     * Set the transmit output power.  Higher power reaches further but draws
     * more current from the battery.  Both ends do not need to match.
     * @param dBm transmit power in dBm, between 2 and 20, eg: 17
     */
    //% block="LoRa set TX power $dBm dBm"
    //% dBm.min=2 dBm.max=20
    //% group="Configuration" advanced=true weight=90
    export function setTxPower(dBm: number): void {
        if (!_inited) return
        if (dBm == 20) {
            writeReg(REG_PA_CONFIG, 0x80 | 0x0F)
            writeReg(REG_PA_DAC, 0x87)
        } else {
            const power = Math.max(0, Math.min(15, dBm - 2))
            writeReg(REG_PA_CONFIG, 0x80 | power)
            writeReg(REG_PA_DAC, 0x84)
        }
    }

    /**
     * Set the LoRa spreading factor.  Higher spreading factors give longer
     * range at the cost of slower transmission.  Both the sender and receiver
     * must use the same spreading factor or they will not be able to
     * communicate.  Default is SF7.
     * @param sf the spreading factor to use, eg: Lora.SpreadingFactor.SF7
     */
    //% block="LoRa set spreading factor $sf"
    //% group="Configuration" advanced=true weight=80
    export function setSpreadingFactor(sf: SpreadingFactor): void {
        if (!_inited) return
        _sf = sf
        const current = readReg(REG_MODEM_CONFIG_2)
        writeReg(REG_MODEM_CONFIG_2, (current & 0x0F) | (sf << 4))
        applyLdro()
    }

    /**
     * Set the LoRa signal bandwidth.  Narrower bandwidths give better
     * sensitivity and range; wider bandwidths give faster data rates.  Both
     * the sender and receiver must use the same bandwidth.  Default is 125 kHz.
     * @param bw the bandwidth to use, eg: Lora.Bandwidth.BW125k
     */
    //% block="LoRa set bandwidth $bw"
    //% group="Configuration" advanced=true weight=70
    export function setBandwidth(bw: Bandwidth): void {
        if (!_inited) return
        _bwCode = bw
        const current = readReg(REG_MODEM_CONFIG_1)
        writeReg(REG_MODEM_CONFIG_1, (current & 0x0F) | (bw << 4))
        applyLdro()
    }

    /**
     * Set the forward error-correction coding rate.  Higher coding rates add
     * more redundancy so packets survive noisier conditions, at the cost of
     * slightly lower throughput.  Both the sender and receiver must match.
     * Default is 4/5.
     * @param cr the coding rate to use, eg: Lora.CodingRate.CR45
     */
    //% block="LoRa set coding rate $cr"
    //% group="Configuration" advanced=true weight=60
    export function setCodingRate(cr: CodingRate): void {
        if (!_inited) return
        const current = readReg(REG_MODEM_CONFIG_1)
        writeReg(REG_MODEM_CONFIG_1, (current & 0xF1) | (cr << 1))
    }

    /**
     * Set the LoRa sync word.  Only radios sharing the same sync word will
     * receive each other's packets, so this can be used to create isolated
     * networks.  ``0x34`` is the public LoRa default.  Both ends must match.
     * @param word sync word byte (0–255), eg: 0x34
     */
    //% block="LoRa set sync word $word"
    //% word.min=0 word.max=255
    //% group="Configuration" advanced=true weight=50
    export function setSyncWord(word: number): void {
        if (!_inited) return
        writeReg(REG_SYNC_WORD, word)
    }
}
