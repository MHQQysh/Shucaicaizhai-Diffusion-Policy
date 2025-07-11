// Auto-generated. Do not edit!

// (in-package bmirobot_tools.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PointsPR {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Px = null;
      this.Py = null;
      this.Pz = null;
      this.Rx = null;
      this.Ry = null;
      this.Rz = null;
    }
    else {
      if (initObj.hasOwnProperty('Px')) {
        this.Px = initObj.Px
      }
      else {
        this.Px = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('Py')) {
        this.Py = initObj.Py
      }
      else {
        this.Py = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('Pz')) {
        this.Pz = initObj.Pz
      }
      else {
        this.Pz = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('Rx')) {
        this.Rx = initObj.Rx
      }
      else {
        this.Rx = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('Ry')) {
        this.Ry = initObj.Ry
      }
      else {
        this.Ry = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('Rz')) {
        this.Rz = initObj.Rz
      }
      else {
        this.Rz = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PointsPR
    // Check that the constant length array field [Px] has the right length
    if (obj.Px.length !== 4) {
      throw new Error('Unable to serialize array field Px - length must be 4')
    }
    // Serialize message field [Px]
    bufferOffset = _arraySerializer.float64(obj.Px, buffer, bufferOffset, 4);
    // Check that the constant length array field [Py] has the right length
    if (obj.Py.length !== 4) {
      throw new Error('Unable to serialize array field Py - length must be 4')
    }
    // Serialize message field [Py]
    bufferOffset = _arraySerializer.float64(obj.Py, buffer, bufferOffset, 4);
    // Check that the constant length array field [Pz] has the right length
    if (obj.Pz.length !== 4) {
      throw new Error('Unable to serialize array field Pz - length must be 4')
    }
    // Serialize message field [Pz]
    bufferOffset = _arraySerializer.float64(obj.Pz, buffer, bufferOffset, 4);
    // Check that the constant length array field [Rx] has the right length
    if (obj.Rx.length !== 4) {
      throw new Error('Unable to serialize array field Rx - length must be 4')
    }
    // Serialize message field [Rx]
    bufferOffset = _arraySerializer.float64(obj.Rx, buffer, bufferOffset, 4);
    // Check that the constant length array field [Ry] has the right length
    if (obj.Ry.length !== 4) {
      throw new Error('Unable to serialize array field Ry - length must be 4')
    }
    // Serialize message field [Ry]
    bufferOffset = _arraySerializer.float64(obj.Ry, buffer, bufferOffset, 4);
    // Check that the constant length array field [Rz] has the right length
    if (obj.Rz.length !== 4) {
      throw new Error('Unable to serialize array field Rz - length must be 4')
    }
    // Serialize message field [Rz]
    bufferOffset = _arraySerializer.float64(obj.Rz, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PointsPR
    let len;
    let data = new PointsPR(null);
    // Deserialize message field [Px]
    data.Px = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [Py]
    data.Py = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [Pz]
    data.Pz = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [Rx]
    data.Rx = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [Ry]
    data.Ry = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [Rz]
    data.Rz = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    return 192;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bmirobot_tools/PointsPR';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f5a7d3cfad1346be5c8446784c69a6c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[4] Px
    float64[4] Py
    float64[4] Pz
    float64[4] Rx
    float64[4] Ry
    float64[4] Rz
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PointsPR(null);
    if (msg.Px !== undefined) {
      resolved.Px = msg.Px;
    }
    else {
      resolved.Px = new Array(4).fill(0)
    }

    if (msg.Py !== undefined) {
      resolved.Py = msg.Py;
    }
    else {
      resolved.Py = new Array(4).fill(0)
    }

    if (msg.Pz !== undefined) {
      resolved.Pz = msg.Pz;
    }
    else {
      resolved.Pz = new Array(4).fill(0)
    }

    if (msg.Rx !== undefined) {
      resolved.Rx = msg.Rx;
    }
    else {
      resolved.Rx = new Array(4).fill(0)
    }

    if (msg.Ry !== undefined) {
      resolved.Ry = msg.Ry;
    }
    else {
      resolved.Ry = new Array(4).fill(0)
    }

    if (msg.Rz !== undefined) {
      resolved.Rz = msg.Rz;
    }
    else {
      resolved.Rz = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = PointsPR;
