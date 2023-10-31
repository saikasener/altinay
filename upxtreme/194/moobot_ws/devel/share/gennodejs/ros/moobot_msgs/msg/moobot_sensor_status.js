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

class moobot_sensor_status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
      this.state_type = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = '';
      }
      if (initObj.hasOwnProperty('state_type')) {
        this.state_type = initObj.state_type
      }
      else {
        this.state_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type moobot_sensor_status
    // Serialize message field [state]
    bufferOffset = _serializer.string(obj.state, buffer, bufferOffset);
    // Serialize message field [state_type]
    bufferOffset = _serializer.int16(obj.state_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type moobot_sensor_status
    let len;
    let data = new moobot_sensor_status(null);
    // Deserialize message field [state]
    data.state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [state_type]
    data.state_type = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.state);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'moobot_msgs/moobot_sensor_status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '49c3408f94118958df8cb878e67805f5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string state
    int16 state_type
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new moobot_sensor_status(null);
    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = ''
    }

    if (msg.state_type !== undefined) {
      resolved.state_type = msg.state_type;
    }
    else {
      resolved.state_type = 0
    }

    return resolved;
    }
};

module.exports = moobot_sensor_status;
