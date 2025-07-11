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

class Robot_ctr {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mtID = null;
      this.mtmode = null;
      this.mtpst = null;
      this.mtspd = null;
      this.mttq = null;
    }
    else {
      if (initObj.hasOwnProperty('mtID')) {
        this.mtID = initObj.mtID
      }
      else {
        this.mtID = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mtmode')) {
        this.mtmode = initObj.mtmode
      }
      else {
        this.mtmode = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mtpst')) {
        this.mtpst = initObj.mtpst
      }
      else {
        this.mtpst = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mtspd')) {
        this.mtspd = initObj.mtspd
      }
      else {
        this.mtspd = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mttq')) {
        this.mttq = initObj.mttq
      }
      else {
        this.mttq = new Array(9).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Robot_ctr
    // Check that the constant length array field [mtID] has the right length
    if (obj.mtID.length !== 9) {
      throw new Error('Unable to serialize array field mtID - length must be 9')
    }
    // Serialize message field [mtID]
    bufferOffset = _arraySerializer.int32(obj.mtID, buffer, bufferOffset, 9);
    // Check that the constant length array field [mtmode] has the right length
    if (obj.mtmode.length !== 9) {
      throw new Error('Unable to serialize array field mtmode - length must be 9')
    }
    // Serialize message field [mtmode]
    bufferOffset = _arraySerializer.int32(obj.mtmode, buffer, bufferOffset, 9);
    // Check that the constant length array field [mtpst] has the right length
    if (obj.mtpst.length !== 9) {
      throw new Error('Unable to serialize array field mtpst - length must be 9')
    }
    // Serialize message field [mtpst]
    bufferOffset = _arraySerializer.int32(obj.mtpst, buffer, bufferOffset, 9);
    // Check that the constant length array field [mtspd] has the right length
    if (obj.mtspd.length !== 9) {
      throw new Error('Unable to serialize array field mtspd - length must be 9')
    }
    // Serialize message field [mtspd]
    bufferOffset = _arraySerializer.int32(obj.mtspd, buffer, bufferOffset, 9);
    // Check that the constant length array field [mttq] has the right length
    if (obj.mttq.length !== 9) {
      throw new Error('Unable to serialize array field mttq - length must be 9')
    }
    // Serialize message field [mttq]
    bufferOffset = _arraySerializer.int32(obj.mttq, buffer, bufferOffset, 9);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Robot_ctr
    let len;
    let data = new Robot_ctr(null);
    // Deserialize message field [mtID]
    data.mtID = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mtmode]
    data.mtmode = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mtpst]
    data.mtpst = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mtspd]
    data.mtspd = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mttq]
    data.mttq = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    return data;
  }

  static getMessageSize(object) {
    return 180;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bmirobot_msg/Robot_ctr';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0fc2a2db85d9265b43f59ed7bec2ae3c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[9] mtID
    int32[9] mtmode
    int32[9] mtpst
    int32[9] mtspd
    int32[9] mttq
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Robot_ctr(null);
    if (msg.mtID !== undefined) {
      resolved.mtID = msg.mtID;
    }
    else {
      resolved.mtID = new Array(9).fill(0)
    }

    if (msg.mtmode !== undefined) {
      resolved.mtmode = msg.mtmode;
    }
    else {
      resolved.mtmode = new Array(9).fill(0)
    }

    if (msg.mtpst !== undefined) {
      resolved.mtpst = msg.mtpst;
    }
    else {
      resolved.mtpst = new Array(9).fill(0)
    }

    if (msg.mtspd !== undefined) {
      resolved.mtspd = msg.mtspd;
    }
    else {
      resolved.mtspd = new Array(9).fill(0)
    }

    if (msg.mttq !== undefined) {
      resolved.mttq = msg.mttq;
    }
    else {
      resolved.mttq = new Array(9).fill(0)
    }

    return resolved;
    }
};

module.exports = Robot_ctr;
