// Auto-generated. Do not edit!

// (in-package bmirobot_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Robot_jointfd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Joint_fdpst = null;
      this.Joint_fdspd = null;
      this.Joint_fdctr = null;
    }
    else {
      if (initObj.hasOwnProperty('Joint_fdpst')) {
        this.Joint_fdpst = initObj.Joint_fdpst
      }
      else {
        this.Joint_fdpst = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('Joint_fdspd')) {
        this.Joint_fdspd = initObj.Joint_fdspd
      }
      else {
        this.Joint_fdspd = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('Joint_fdctr')) {
        this.Joint_fdctr = initObj.Joint_fdctr
      }
      else {
        this.Joint_fdctr = new Array(9).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Robot_jointfd
    // Check that the constant length array field [Joint_fdpst] has the right length
    if (obj.Joint_fdpst.length !== 9) {
      throw new Error('Unable to serialize array field Joint_fdpst - length must be 9')
    }
    // Serialize message field [Joint_fdpst]
    bufferOffset = _arraySerializer.float32(obj.Joint_fdpst, buffer, bufferOffset, 9);
    // Check that the constant length array field [Joint_fdspd] has the right length
    if (obj.Joint_fdspd.length !== 9) {
      throw new Error('Unable to serialize array field Joint_fdspd - length must be 9')
    }
    // Serialize message field [Joint_fdspd]
    bufferOffset = _arraySerializer.float32(obj.Joint_fdspd, buffer, bufferOffset, 9);
    // Check that the constant length array field [Joint_fdctr] has the right length
    if (obj.Joint_fdctr.length !== 9) {
      throw new Error('Unable to serialize array field Joint_fdctr - length must be 9')
    }
    // Serialize message field [Joint_fdctr]
    bufferOffset = _arraySerializer.float32(obj.Joint_fdctr, buffer, bufferOffset, 9);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Robot_jointfd
    let len;
    let data = new Robot_jointfd(null);
    // Deserialize message field [Joint_fdpst]
    data.Joint_fdpst = _arrayDeserializer.float32(buffer, bufferOffset, 9)
    // Deserialize message field [Joint_fdspd]
    data.Joint_fdspd = _arrayDeserializer.float32(buffer, bufferOffset, 9)
    // Deserialize message field [Joint_fdctr]
    data.Joint_fdctr = _arrayDeserializer.float32(buffer, bufferOffset, 9)
    return data;
  }

  static getMessageSize(object) {
    return 108;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bmirobot_msg/Robot_jointfd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8b60b3db716c9968ae69daf16554d81f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[9] Joint_fdpst
    float32[9] Joint_fdspd
    float32[9] Joint_fdctr
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Robot_jointfd(null);
    if (msg.Joint_fdpst !== undefined) {
      resolved.Joint_fdpst = msg.Joint_fdpst;
    }
    else {
      resolved.Joint_fdpst = new Array(9).fill(0)
    }

    if (msg.Joint_fdspd !== undefined) {
      resolved.Joint_fdspd = msg.Joint_fdspd;
    }
    else {
      resolved.Joint_fdspd = new Array(9).fill(0)
    }

    if (msg.Joint_fdctr !== undefined) {
      resolved.Joint_fdctr = msg.Joint_fdctr;
    }
    else {
      resolved.Joint_fdctr = new Array(9).fill(0)
    }

    return resolved;
    }
};

module.exports = Robot_jointfd;
