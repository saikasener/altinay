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
      this.battery_volt_1 = null;
      this.battery_volt_2 = null;
      this.battery_volt_total = null;
      this.battery_temp_1 = null;
      this.battery_temp_2 = null;
      this.current_main = null;
      this.total_power_wh = null;
    }
    else {
      if (initObj.hasOwnProperty('battery_volt_1')) {
        this.battery_volt_1 = initObj.battery_volt_1
      }
      else {
        this.battery_volt_1 = 0.0;
      }
      if (initObj.hasOwnProperty('battery_volt_2')) {
        this.battery_volt_2 = initObj.battery_volt_2
      }
      else {
        this.battery_volt_2 = 0.0;
      }
      if (initObj.hasOwnProperty('battery_volt_total')) {
        this.battery_volt_total = initObj.battery_volt_total
      }
      else {
        this.battery_volt_total = 0.0;
      }
      if (initObj.hasOwnProperty('battery_temp_1')) {
        this.battery_temp_1 = initObj.battery_temp_1
      }
      else {
        this.battery_temp_1 = 0.0;
      }
      if (initObj.hasOwnProperty('battery_temp_2')) {
        this.battery_temp_2 = initObj.battery_temp_2
      }
      else {
        this.battery_temp_2 = 0.0;
      }
      if (initObj.hasOwnProperty('current_main')) {
        this.current_main = initObj.current_main
      }
      else {
        this.current_main = 0.0;
      }
      if (initObj.hasOwnProperty('total_power_wh')) {
        this.total_power_wh = initObj.total_power_wh
      }
      else {
        this.total_power_wh = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type bms_status
    // Serialize message field [battery_volt_1]
    bufferOffset = _serializer.float32(obj.battery_volt_1, buffer, bufferOffset);
    // Serialize message field [battery_volt_2]
    bufferOffset = _serializer.float32(obj.battery_volt_2, buffer, bufferOffset);
    // Serialize message field [battery_volt_total]
    bufferOffset = _serializer.float32(obj.battery_volt_total, buffer, bufferOffset);
    // Serialize message field [battery_temp_1]
    bufferOffset = _serializer.float32(obj.battery_temp_1, buffer, bufferOffset);
    // Serialize message field [battery_temp_2]
    bufferOffset = _serializer.float32(obj.battery_temp_2, buffer, bufferOffset);
    // Serialize message field [current_main]
    bufferOffset = _serializer.float32(obj.current_main, buffer, bufferOffset);
    // Serialize message field [total_power_wh]
    bufferOffset = _serializer.float32(obj.total_power_wh, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type bms_status
    let len;
    let data = new bms_status(null);
    // Deserialize message field [battery_volt_1]
    data.battery_volt_1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [battery_volt_2]
    data.battery_volt_2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [battery_volt_total]
    data.battery_volt_total = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [battery_temp_1]
    data.battery_temp_1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [battery_temp_2]
    data.battery_temp_2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_main]
    data.current_main = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [total_power_wh]
    data.total_power_wh = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'moobot_msgs/bms_status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3113256c57eafd1210064dbad908f61d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 battery_volt_1
    float32 battery_volt_2
    float32 battery_volt_total
    float32 battery_temp_1
    float32 battery_temp_2
    float32 current_main
    float32 total_power_wh
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new bms_status(null);
    if (msg.battery_volt_1 !== undefined) {
      resolved.battery_volt_1 = msg.battery_volt_1;
    }
    else {
      resolved.battery_volt_1 = 0.0
    }

    if (msg.battery_volt_2 !== undefined) {
      resolved.battery_volt_2 = msg.battery_volt_2;
    }
    else {
      resolved.battery_volt_2 = 0.0
    }

    if (msg.battery_volt_total !== undefined) {
      resolved.battery_volt_total = msg.battery_volt_total;
    }
    else {
      resolved.battery_volt_total = 0.0
    }

    if (msg.battery_temp_1 !== undefined) {
      resolved.battery_temp_1 = msg.battery_temp_1;
    }
    else {
      resolved.battery_temp_1 = 0.0
    }

    if (msg.battery_temp_2 !== undefined) {
      resolved.battery_temp_2 = msg.battery_temp_2;
    }
    else {
      resolved.battery_temp_2 = 0.0
    }

    if (msg.current_main !== undefined) {
      resolved.current_main = msg.current_main;
    }
    else {
      resolved.current_main = 0.0
    }

    if (msg.total_power_wh !== undefined) {
      resolved.total_power_wh = msg.total_power_wh;
    }
    else {
      resolved.total_power_wh = 0.0
    }

    return resolved;
    }
};

module.exports = bms_status;
