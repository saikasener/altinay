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

class conveyor_status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.load_conveyor = null;
      this.unload_conveyor = null;
    }
    else {
      if (initObj.hasOwnProperty('load_conveyor')) {
        this.load_conveyor = initObj.load_conveyor
      }
      else {
        this.load_conveyor = false;
      }
      if (initObj.hasOwnProperty('unload_conveyor')) {
        this.unload_conveyor = initObj.unload_conveyor
      }
      else {
        this.unload_conveyor = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type conveyor_status
    // Serialize message field [load_conveyor]
    bufferOffset = _serializer.bool(obj.load_conveyor, buffer, bufferOffset);
    // Serialize message field [unload_conveyor]
    bufferOffset = _serializer.bool(obj.unload_conveyor, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type conveyor_status
    let len;
    let data = new conveyor_status(null);
    // Deserialize message field [load_conveyor]
    data.load_conveyor = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [unload_conveyor]
    data.unload_conveyor = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'moobot_msgs/conveyor_status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8462790025e5d19ec5df8dbb71cb071';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool load_conveyor
    bool unload_conveyor
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new conveyor_status(null);
    if (msg.load_conveyor !== undefined) {
      resolved.load_conveyor = msg.load_conveyor;
    }
    else {
      resolved.load_conveyor = false
    }

    if (msg.unload_conveyor !== undefined) {
      resolved.unload_conveyor = msg.unload_conveyor;
    }
    else {
      resolved.unload_conveyor = false
    }

    return resolved;
    }
};

module.exports = conveyor_status;
