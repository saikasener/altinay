// Auto-generated. Do not edit!

// (in-package moobot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class bms_status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.battery_voltage = null;
      this.load_voltage = null;
      this.charger_voltage = null;
      this.cell_voltage = null;
      this.temperature_bms = null;
      this.temperature_cell = null;
      this.current_income = null;
      this.capacity = null;
      this.capacity_max = null;
      this.soc = null;
      this.current_charger = null;
      this.current_load = null;
    }
    else {
      if (initObj.hasOwnProperty('battery_voltage')) {
        this.battery_voltage = initObj.battery_voltage
      }
      else {
        this.battery_voltage = 0.0;
      }
      if (initObj.hasOwnProperty('load_voltage')) {
        this.load_voltage = initObj.load_voltage
      }
      else {
        this.load_voltage = 0.0;
      }
      if (initObj.hasOwnProperty('charger_voltage')) {
        this.charger_voltage = initObj.charger_voltage
      }
      else {
        this.charger_voltage = 0.0;
      }
      if (initObj.hasOwnProperty('cell_voltage')) {
        this.cell_voltage = initObj.cell_voltage
      }
      else {
        this.cell_voltage = [];
      }
      if (initObj.hasOwnProperty('temperature_bms')) {
        this.temperature_bms = initObj.temperature_bms
      }
      else {
        this.temperature_bms = 0.0;
      }
      if (initObj.hasOwnProperty('temperature_cell')) {
        this.temperature_cell = initObj.temperature_cell
      }
      else {
        this.temperature_cell = [];
      }
      if (initObj.hasOwnProperty('current_income')) {
        this.current_income = initObj.current_income
      }
      else {
        this.current_income = 0.0;
      }
      if (initObj.hasOwnProperty('capacity')) {
        this.capacity = initObj.capacity
      }
      else {
        this.capacity = 0.0;
      }
      if (initObj.hasOwnProperty('capacity_max')) {
        this.capacity_max = initObj.capacity_max
      }
      else {
        this.capacity_max = 0.0;
      }
      if (initObj.hasOwnProperty('soc')) {
        this.soc = initObj.soc
      }
      else {
        this.soc = 0.0;
      }
      if (initObj.hasOwnProperty('current_charger')) {
        this.current_charger = initObj.current_charger
      }
      else {
        this.current_charger = 0.0;
      }
      if (initObj.hasOwnProperty('current_load')) {
        this.current_load = initObj.current_load
      }
      else {
        this.current_load = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type bms_status
    // Serialize message field [battery_voltage]
    bufferOffset = _serializer.float32(obj.battery_voltage, buffer, bufferOffset);
    // Serialize message field [load_voltage]
    bufferOffset = _serializer.float32(obj.load_voltage, buffer, bufferOffset);
    // Serialize message field [charger_voltage]
    bufferOffset = _serializer.float32(obj.charger_voltage, buffer, bufferOffset);
    // Serialize message field [cell_voltage]
    bufferOffset = _arraySerializer.float32(obj.cell_voltage, buffer, bufferOffset, null);
    // Serialize message field [temperature_bms]
    bufferOffset = _serializer.float32(obj.temperature_bms, buffer, bufferOffset);
    // Serialize message field [temperature_cell]
    bufferOffset = _arraySerializer.float32(obj.temperature_cell, buffer, bufferOffset, null);
    // Serialize message field [current_income]
    bufferOffset = _serializer.float32(obj.current_income, buffer, bufferOffset);
    // Serialize message field [capacity]
    bufferOffset = _serializer.float32(obj.capacity, buffer, bufferOffset);
    // Serialize message field [capacity_max]
    bufferOffset = _serializer.float32(obj.capacity_max, buffer, bufferOffset);
    // Serialize message field [soc]
    bufferOffset = _serializer.float32(obj.soc, buffer, bufferOffset);
    // Serialize message field [current_charger]
    bufferOffset = _serializer.float32(obj.current_charger, buffer, bufferOffset);
    // Serialize message field [current_load]
    bufferOffset = _serializer.float32(obj.current_load, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type bms_status
    let len;
    let data = new bms_status(null);
    // Deserialize message field [battery_voltage]
    data.battery_voltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [load_voltage]
    data.load_voltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [charger_voltage]
    data.charger_voltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cell_voltage]
    data.cell_voltage = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [temperature_bms]
    data.temperature_bms = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [temperature_cell]
    data.temperature_cell = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [current_income]
    data.current_income = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [capacity]
    data.capacity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [capacity_max]
    data.capacity_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [soc]
    data.soc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_charger]
    data.current_charger = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_load]
    data.current_load = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.cell_voltage.length;
    length += 4 * object.temperature_cell.length;
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'moobot_msgs/bms_status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '08c0d01e45dd995f3adfffdf7aa31d50';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 battery_voltage  # Voltage in Volts 
    float32 load_voltage  # Voltage in Volts 
    float32 charger_voltage  # Voltage in Volts 
    float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
    
    float32 temperature_bms      # Temperature in Degrees Celsius (If unmeasured NaN)
    float32[] temperature_cell  # An array of individual cell temperatures for each cell in the pack
    
    float32 current_income     # Pile gelen akım      
    float32 capacity           # Pilin doluluk kapasitesi -> Capacity in Ah (last full capacity) 
    float32 capacity_max  	    # Pilin total kapasitesi -> Capacity in Ah (design capacity)  
    float32 soc                # Charge percentage on 0 to 1 range -> percentage
    float32 current_charger    # A     
    float32 current_load       # Sistemin Çektiği Akım -> Negative when discharging (A)  
    
    #float32 battery_volt_1
    #float32 battery_volt_2
    #float32 battery_volt_total
    #float32 battery_temp_1
    #float32 battery_temp_2
    #float32 current_main
    #float32 total_power_wh
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new bms_status(null);
    if (msg.battery_voltage !== undefined) {
      resolved.battery_voltage = msg.battery_voltage;
    }
    else {
      resolved.battery_voltage = 0.0
    }

    if (msg.load_voltage !== undefined) {
      resolved.load_voltage = msg.load_voltage;
    }
    else {
      resolved.load_voltage = 0.0
    }

    if (msg.charger_voltage !== undefined) {
      resolved.charger_voltage = msg.charger_voltage;
    }
    else {
      resolved.charger_voltage = 0.0
    }

    if (msg.cell_voltage !== undefined) {
      resolved.cell_voltage = msg.cell_voltage;
    }
    else {
      resolved.cell_voltage = []
    }

    if (msg.temperature_bms !== undefined) {
      resolved.temperature_bms = msg.temperature_bms;
    }
    else {
      resolved.temperature_bms = 0.0
    }

    if (msg.temperature_cell !== undefined) {
      resolved.temperature_cell = msg.temperature_cell;
    }
    else {
      resolved.temperature_cell = []
    }

    if (msg.current_income !== undefined) {
      resolved.current_income = msg.current_income;
    }
    else {
      resolved.current_income = 0.0
    }

    if (msg.capacity !== undefined) {
      resolved.capacity = msg.capacity;
    }
    else {
      resolved.capacity = 0.0
    }

    if (msg.capacity_max !== undefined) {
      resolved.capacity_max = msg.capacity_max;
    }
    else {
      resolved.capacity_max = 0.0
    }

    if (msg.soc !== undefined) {
      resolved.soc = msg.soc;
    }
    else {
      resolved.soc = 0.0
    }

    if (msg.current_charger !== undefined) {
      resolved.current_charger = msg.current_charger;
    }
    else {
      resolved.current_charger = 0.0
    }

    if (msg.current_load !== undefined) {
      resolved.current_load = msg.current_load;
    }
    else {
      resolved.current_load = 0.0
    }

    return resolved;
    }
};

module.exports = bms_status;
