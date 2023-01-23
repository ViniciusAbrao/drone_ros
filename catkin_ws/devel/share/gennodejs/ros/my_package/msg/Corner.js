// Auto-generated. Do not edit!

// (in-package my_package.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Corner {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.corner_x = null;
      this.corner_y = null;
      this.corner_z = null;
    }
    else {
      if (initObj.hasOwnProperty('corner_x')) {
        this.corner_x = initObj.corner_x
      }
      else {
        this.corner_x = 0.0;
      }
      if (initObj.hasOwnProperty('corner_y')) {
        this.corner_y = initObj.corner_y
      }
      else {
        this.corner_y = 0.0;
      }
      if (initObj.hasOwnProperty('corner_z')) {
        this.corner_z = initObj.corner_z
      }
      else {
        this.corner_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Corner
    // Serialize message field [corner_x]
    bufferOffset = _serializer.float32(obj.corner_x, buffer, bufferOffset);
    // Serialize message field [corner_y]
    bufferOffset = _serializer.float32(obj.corner_y, buffer, bufferOffset);
    // Serialize message field [corner_z]
    bufferOffset = _serializer.float32(obj.corner_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Corner
    let len;
    let data = new Corner(null);
    // Deserialize message field [corner_x]
    data.corner_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [corner_y]
    data.corner_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [corner_z]
    data.corner_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_package/Corner';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '92060748d9174f56becd0c34fba143e6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 corner_x
    float32 corner_y
    float32 corner_z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Corner(null);
    if (msg.corner_x !== undefined) {
      resolved.corner_x = msg.corner_x;
    }
    else {
      resolved.corner_x = 0.0
    }

    if (msg.corner_y !== undefined) {
      resolved.corner_y = msg.corner_y;
    }
    else {
      resolved.corner_y = 0.0
    }

    if (msg.corner_z !== undefined) {
      resolved.corner_z = msg.corner_z;
    }
    else {
      resolved.corner_z = 0.0
    }

    return resolved;
    }
};

module.exports = Corner;
