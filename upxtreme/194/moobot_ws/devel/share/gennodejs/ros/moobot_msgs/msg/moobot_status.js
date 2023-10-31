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

class moobot_status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.agv_mode = null;
      this.agv_stopped = null;
      this.emergency = null;
      this.fault = null;
      this.job = null;
    }
    else {
      if (initObj.hasOwnProperty('agv_mode')) {
        this.agv_mode = initObj.agv_mode
      }
      else {
        this.agv_mode = 0;
      }
      if (initObj.hasOwnProperty('agv_stopped')) {
        this.agv_stopped = initObj.agv_stopped
      }
      else {
        this.agv_stopped = false;
      }
      if (initObj.hasOwnProperty('emergency')) {
        this.emergency = initObj.emergency
      }
      else {
        this.emergency = false;
      }
      if (initObj.hasOwnProperty('fault')) {
        this.fault = initObj.fault
      }
      else {
        this.fault = false;
      }
      if (initObj.hasOwnProperty('job')) {
        this.job = initObj.job
      }
      else {
        this.job = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type moobot_status
    // Serialize message field [agv_mode]
    bufferOffset = _serializer.int32(obj.agv_mode, buffer, bufferOffset);
    // Serialize message field [agv_stopped]
    bufferOffset = _serializer.bool(obj.agv_stopped, buffer, bufferOffset);
    // Serialize message field [emergency]
    bufferOffset = _serializer.bool(obj.emergency, buffer, bufferOffset);
    // Serialize message field [fault]
    bufferOffset = _serializer.bool(obj.fault, buffer, bufferOffset);
    // Serialize message field [job]
    bufferOffset = _serializer.string(obj.job, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type moobot_status
    let len;
    let data = new moobot_status(null);
    // Deserialize message field [agv_mode]
    data.agv_mode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [agv_stopped]
    data.agv_stopped = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [emergency]
    data.emergency = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fault]
    data.fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [job]
    data.job = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.job);
    return length + 11;
  }

  static datatype() {
    // Returns string type for a message object
    return 'moobot_msgs/moobot_status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6ac369f66a5b4dcf60abbe45c6d5c34f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 agv_mode
    bool agv_stopped
    bool emergency
    bool fault
    string job
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new moobot_status(null);
    if (msg.agv_mode !== undefined) {
      resolved.agv_mode = msg.agv_mode;
    }
    else {
      resolved.agv_mode = 0
    }

    if (msg.agv_stopped !== undefined) {
      resolved.agv_stopped = msg.agv_stopped;
    }
    else {
      resolved.agv_stopped = false
    }

    if (msg.emergency !== undefined) {
      resolved.emergency = msg.emergency;
    }
    else {
      resolved.emergency = false
    }

    if (msg.fault !== undefined) {
      resolved.fault = msg.fault;
    }
    else {
      resolved.fault = false
    }

    if (msg.job !== undefined) {
      resolved.job = msg.job;
    }
    else {
      resolved.job = ''
    }

    return resolved;
    }
};

module.exports = moobot_status;
