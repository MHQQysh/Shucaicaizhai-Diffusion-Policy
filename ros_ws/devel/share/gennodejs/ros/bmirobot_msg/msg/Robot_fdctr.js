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

class Robot_fdctr {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mt_mode = null;
      this.mt_Cpst = null;
      this.mt_Cspd = null;
      this.mt_incrt = null;
      this.mt_PWMduty = null;
    }
    else {
      if (initObj.hasOwnProperty('mt_mode')) {
        this.mt_mode = initObj.mt_mode
      }
      else {
        this.mt_mode = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_Cpst')) {
        this.mt_Cpst = initObj.mt_Cpst
      }
      else {
        this.mt_Cpst = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_Cspd')) {
        this.mt_Cspd = initObj.mt_Cspd
      }
      else {
        this.mt_Cspd = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_incrt')) {
        this.mt_incrt = initObj.mt_incrt
      }
      else {
        this.mt_incrt = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_PWMduty')) {
        this.mt_PWMduty = initObj.mt_PWMduty
      }
      else {
        this.mt_PWMduty = new Array(9).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Robot_fdctr
    // Check that the constant length array field [mt_mode] has the right length
    if (obj.mt_mode.length !== 9) {
      throw new Error('Unable to serialize array field mt_mode - length must be 9')
    }
    // Serialize message field [mt_mode]
    bufferOffset = _arraySerializer.int32(obj.mt_mode, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_Cpst] has the right length
    if (obj.mt_Cpst.length !== 9) {
      throw new Error('Unable to serialize array field mt_Cpst - length must be 9')
    }
    // Serialize message field [mt_Cpst]
    bufferOffset = _arraySerializer.int32(obj.mt_Cpst, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_Cspd] has the right length
    if (obj.mt_Cspd.length !== 9) {
      throw new Error('Unable to serialize array field mt_Cspd - length must be 9')
    }
    // Serialize message field [mt_Cspd]
    bufferOffset = _arraySerializer.int32(obj.mt_Cspd, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_incrt] has the right length
    if (obj.mt_incrt.length !== 9) {
      throw new Error('Unable to serialize array field mt_incrt - length must be 9')
    }
    // Serialize message field [mt_incrt]
    bufferOffset = _arraySerializer.int32(obj.mt_incrt, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_PWMduty] has the right length
    if (obj.mt_PWMduty.length !== 9) {
      throw new Error('Unable to serialize array field mt_PWMduty - length must be 9')
    }
    // Serialize message field [mt_PWMduty]
    bufferOffset = _arraySerializer.int32(obj.mt_PWMduty, buffer, bufferOffset, 9);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Robot_fdctr
    let len;
    let data = new Robot_fdctr(null);
    // Deserialize message field [mt_mode]
    data.mt_mode = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_Cpst]
    data.mt_Cpst = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_Cspd]
    data.mt_Cspd = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_incrt]
    data.mt_incrt = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_PWMduty]
    data.mt_PWMduty = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    return data;
  }

  static getMessageSize(object) {
    return 180;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bmirobot_msg/Robot_fdctr';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3195cf007256e8ddfd66fc62f56f5233';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[9] mt_mode
    int32[9] mt_Cpst
    int32[9] mt_Cspd
    int32[9] mt_incrt
    int32[9] mt_PWMduty
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Robot_fdctr(null);
    if (msg.mt_mode !== undefined) {
      resolved.mt_mode = msg.mt_mode;
    }
    else {
      resolved.mt_mode = new Array(9).fill(0)
    }

    if (msg.mt_Cpst !== undefined) {
      resolved.mt_Cpst = msg.mt_Cpst;
    }
    else {
      resolved.mt_Cpst = new Array(9).fill(0)
    }

    if (msg.mt_Cspd !== undefined) {
      resolved.mt_Cspd = msg.mt_Cspd;
    }
    else {
      resolved.mt_Cspd = new Array(9).fill(0)
    }

    if (msg.mt_incrt !== undefined) {
      resolved.mt_incrt = msg.mt_incrt;
    }
    else {
      resolved.mt_incrt = new Array(9).fill(0)
    }

    if (msg.mt_PWMduty !== undefined) {
      resolved.mt_PWMduty = msg.mt_PWMduty;
    }
    else {
      resolved.mt_PWMduty = new Array(9).fill(0)
    }

    return resolved;
    }
};

module.exports = Robot_fdctr;
