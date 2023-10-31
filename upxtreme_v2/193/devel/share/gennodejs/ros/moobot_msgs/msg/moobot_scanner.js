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

class moobot_scanner {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fs_ossd = null;
      this.fs_w1 = null;
      this.fs_w2 = null;
      this.bs_ossd = null;
      this.bs_w1 = null;
      this.bs_w2 = null;
    }
    else {
      if (initObj.hasOwnProperty('fs_ossd')) {
        this.fs_ossd = initObj.fs_ossd
      }
      else {
        this.fs_ossd = 0;
      }
      if (initObj.hasOwnProperty('fs_w1')) {
        this.fs_w1 = initObj.fs_w1
      }
      else {
        this.fs_w1 = 0;
      }
      if (initObj.hasOwnProperty('fs_w2')) {
        this.fs_w2 = initObj.fs_w2
      }
      else {
        this.fs_w2 = 0;
      }
      if (initObj.hasOwnProperty('bs_ossd')) {
        this.bs_ossd = initObj.bs_ossd
      }
      else {
        this.bs_ossd = 0;
      }
      if (initObj.hasOwnProperty('bs_w1')) {
        this.bs_w1 = initObj.bs_w1
      }
      else {
        this.bs_w1 = 0;
      }
      if (initObj.hasOwnProperty('bs_w2')) {
        this.bs_w2 = initObj.bs_w2
      }
      else {
        this.bs_w2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type moobot_scanner
    // Serialize message field [fs_ossd]
    bufferOffset = _serializer.byte(obj.fs_ossd, buffer, bufferOffset);
    // Serialize message field [fs_w1]
    bufferOffset = _serializer.byte(obj.fs_w1, buffer, bufferOffset);
    // Serialize message field [fs_w2]
    bufferOffset = _serializer.byte(obj.fs_w2, buffer, bufferOffset);
    // Serialize message field [bs_ossd]
    bufferOffset = _serializer.byte(obj.bs_ossd, buffer, bufferOffset);
    // Serialize message field [bs_w1]
    bufferOffset = _serializer.byte(obj.bs_w1, buffer, bufferOffset);
    // Serialize message field [bs_w2]
    bufferOffset = _serializer.byte(obj.bs_w2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type moobot_scanner
    let len;
    let data = new moobot_scanner(null);
    // Deserialize message field [fs_ossd]
    data.fs_ossd = _deserializer.byte(buffer, bufferOffset);
    // Deserialize message field [fs_w1]
    data.fs_w1 = _deserializer.byte(buffer, bufferOffset);
    // Deserialize message field [fs_w2]
    data.fs_w2 = _deserializer.byte(buffer, bufferOffset);
    // Deserialize message field [bs_ossd]
    data.bs_ossd = _deserializer.byte(buffer, bufferOffset);
    // Deserialize message field [bs_w1]
    data.bs_w1 = _deserializer.byte(buffer, bufferOffset);
    // Deserialize message field [bs_w2]
    data.bs_w2 = _deserializer.byte(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'moobot_msgs/moobot_scanner';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b971113dac55cb95f6b07a91c6331c49';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    byte fs_ossd
    byte fs_w1
    byte fs_w2
    byte bs_ossd
    byte bs_w1
    byte bs_w2
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new moobot_scanner(null);
    if (msg.fs_ossd !== undefined) {
      resolved.fs_ossd = msg.fs_ossd;
    }
    else {
      resolved.fs_ossd = 0
    }

    if (msg.fs_w1 !== undefined) {
      resolved.fs_w1 = msg.fs_w1;
    }
    else {
      resolved.fs_w1 = 0
    }

    if (msg.fs_w2 !== undefined) {
      resolved.fs_w2 = msg.fs_w2;
    }
    else {
      resolved.fs_w2 = 0
    }

    if (msg.bs_ossd !== undefined) {
      resolved.bs_ossd = msg.bs_ossd;
    }
    else {
      resolved.bs_ossd = 0
    }

    if (msg.bs_w1 !== undefined) {
      resolved.bs_w1 = msg.bs_w1;
    }
    else {
      resolved.bs_w1 = 0
    }

    if (msg.bs_w2 !== undefined) {
      resolved.bs_w2 = msg.bs_w2;
    }
    else {
      resolved.bs_w2 = 0
    }

    return resolved;
    }
};

module.exports = moobot_scanner;
