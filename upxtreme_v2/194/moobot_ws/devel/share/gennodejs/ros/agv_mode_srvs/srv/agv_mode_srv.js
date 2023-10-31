// Auto-generated. Do not edit!

// (in-package agv_mode_srvs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class agv_mode_srvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type agv_mode_srvRequest
    // Serialize message field [mode]
    bufferOffset = _serializer.int64(obj.mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type agv_mode_srvRequest
    let len;
    let data = new agv_mode_srvRequest(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'agv_mode_srvs/agv_mode_srvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '284404659b502753974e60f7457ed2dc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 mode
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new agv_mode_srvRequest(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    return resolved;
    }
};

class agv_mode_srvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.newmode = null;
    }
    else {
      if (initObj.hasOwnProperty('newmode')) {
        this.newmode = initObj.newmode
      }
      else {
        this.newmode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type agv_mode_srvResponse
    // Serialize message field [newmode]
    bufferOffset = _serializer.int64(obj.newmode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type agv_mode_srvResponse
    let len;
    let data = new agv_mode_srvResponse(null);
    // Deserialize message field [newmode]
    data.newmode = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'agv_mode_srvs/agv_mode_srvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '961eb286071c8202bbb98ce138adb626';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 newmode
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new agv_mode_srvResponse(null);
    if (msg.newmode !== undefined) {
      resolved.newmode = msg.newmode;
    }
    else {
      resolved.newmode = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: agv_mode_srvRequest,
  Response: agv_mode_srvResponse,
  md5sum() { return 'ae96190e987c19063498c13c3886f347'; },
  datatype() { return 'agv_mode_srvs/agv_mode_srv'; }
};
