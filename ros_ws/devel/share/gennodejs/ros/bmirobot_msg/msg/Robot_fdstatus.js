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

class Robot_fdstatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mt_mode = null;
      this.mt_Gpst = null;
      this.mt_Cpst = null;
      this.mt_Lpst = null;
      this.mt_Gspd = null;
      this.mt_Cspd = null;
      this.mt_Lspd = null;
      this.mt_Gtq = null;
      this.mt_PWMduty = null;
      this.mt_Gtime = null;
      this.mt_Ctime = null;
      this.mt_Rtime = null;
      this.mt_sysclk = null;
      this.mt_smptime = null;
      this.mt_cputmp = null;
      this.mt_mttmp = null;
      this.mt_invlt = null;
      this.mt_incrt = null;
      this.mt_PWMfrq = null;
      this.mt_ecd = null;
      this.mt_ecdcnt = null;
    }
    else {
      if (initObj.hasOwnProperty('mt_mode')) {
        this.mt_mode = initObj.mt_mode
      }
      else {
        this.mt_mode = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_Gpst')) {
        this.mt_Gpst = initObj.mt_Gpst
      }
      else {
        this.mt_Gpst = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_Cpst')) {
        this.mt_Cpst = initObj.mt_Cpst
      }
      else {
        this.mt_Cpst = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_Lpst')) {
        this.mt_Lpst = initObj.mt_Lpst
      }
      else {
        this.mt_Lpst = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_Gspd')) {
        this.mt_Gspd = initObj.mt_Gspd
      }
      else {
        this.mt_Gspd = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_Cspd')) {
        this.mt_Cspd = initObj.mt_Cspd
      }
      else {
        this.mt_Cspd = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_Lspd')) {
        this.mt_Lspd = initObj.mt_Lspd
      }
      else {
        this.mt_Lspd = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_Gtq')) {
        this.mt_Gtq = initObj.mt_Gtq
      }
      else {
        this.mt_Gtq = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_PWMduty')) {
        this.mt_PWMduty = initObj.mt_PWMduty
      }
      else {
        this.mt_PWMduty = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_Gtime')) {
        this.mt_Gtime = initObj.mt_Gtime
      }
      else {
        this.mt_Gtime = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_Ctime')) {
        this.mt_Ctime = initObj.mt_Ctime
      }
      else {
        this.mt_Ctime = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_Rtime')) {
        this.mt_Rtime = initObj.mt_Rtime
      }
      else {
        this.mt_Rtime = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_sysclk')) {
        this.mt_sysclk = initObj.mt_sysclk
      }
      else {
        this.mt_sysclk = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_smptime')) {
        this.mt_smptime = initObj.mt_smptime
      }
      else {
        this.mt_smptime = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_cputmp')) {
        this.mt_cputmp = initObj.mt_cputmp
      }
      else {
        this.mt_cputmp = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_mttmp')) {
        this.mt_mttmp = initObj.mt_mttmp
      }
      else {
        this.mt_mttmp = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_invlt')) {
        this.mt_invlt = initObj.mt_invlt
      }
      else {
        this.mt_invlt = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_incrt')) {
        this.mt_incrt = initObj.mt_incrt
      }
      else {
        this.mt_incrt = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_PWMfrq')) {
        this.mt_PWMfrq = initObj.mt_PWMfrq
      }
      else {
        this.mt_PWMfrq = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_ecd')) {
        this.mt_ecd = initObj.mt_ecd
      }
      else {
        this.mt_ecd = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('mt_ecdcnt')) {
        this.mt_ecdcnt = initObj.mt_ecdcnt
      }
      else {
        this.mt_ecdcnt = new Array(9).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Robot_fdstatus
    // Check that the constant length array field [mt_mode] has the right length
    if (obj.mt_mode.length !== 9) {
      throw new Error('Unable to serialize array field mt_mode - length must be 9')
    }
    // Serialize message field [mt_mode]
    bufferOffset = _arraySerializer.int32(obj.mt_mode, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_Gpst] has the right length
    if (obj.mt_Gpst.length !== 9) {
      throw new Error('Unable to serialize array field mt_Gpst - length must be 9')
    }
    // Serialize message field [mt_Gpst]
    bufferOffset = _arraySerializer.int32(obj.mt_Gpst, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_Cpst] has the right length
    if (obj.mt_Cpst.length !== 9) {
      throw new Error('Unable to serialize array field mt_Cpst - length must be 9')
    }
    // Serialize message field [mt_Cpst]
    bufferOffset = _arraySerializer.int32(obj.mt_Cpst, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_Lpst] has the right length
    if (obj.mt_Lpst.length !== 9) {
      throw new Error('Unable to serialize array field mt_Lpst - length must be 9')
    }
    // Serialize message field [mt_Lpst]
    bufferOffset = _arraySerializer.int32(obj.mt_Lpst, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_Gspd] has the right length
    if (obj.mt_Gspd.length !== 9) {
      throw new Error('Unable to serialize array field mt_Gspd - length must be 9')
    }
    // Serialize message field [mt_Gspd]
    bufferOffset = _arraySerializer.int32(obj.mt_Gspd, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_Cspd] has the right length
    if (obj.mt_Cspd.length !== 9) {
      throw new Error('Unable to serialize array field mt_Cspd - length must be 9')
    }
    // Serialize message field [mt_Cspd]
    bufferOffset = _arraySerializer.int32(obj.mt_Cspd, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_Lspd] has the right length
    if (obj.mt_Lspd.length !== 9) {
      throw new Error('Unable to serialize array field mt_Lspd - length must be 9')
    }
    // Serialize message field [mt_Lspd]
    bufferOffset = _arraySerializer.int32(obj.mt_Lspd, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_Gtq] has the right length
    if (obj.mt_Gtq.length !== 9) {
      throw new Error('Unable to serialize array field mt_Gtq - length must be 9')
    }
    // Serialize message field [mt_Gtq]
    bufferOffset = _arraySerializer.int32(obj.mt_Gtq, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_PWMduty] has the right length
    if (obj.mt_PWMduty.length !== 9) {
      throw new Error('Unable to serialize array field mt_PWMduty - length must be 9')
    }
    // Serialize message field [mt_PWMduty]
    bufferOffset = _arraySerializer.int32(obj.mt_PWMduty, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_Gtime] has the right length
    if (obj.mt_Gtime.length !== 9) {
      throw new Error('Unable to serialize array field mt_Gtime - length must be 9')
    }
    // Serialize message field [mt_Gtime]
    bufferOffset = _arraySerializer.int32(obj.mt_Gtime, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_Ctime] has the right length
    if (obj.mt_Ctime.length !== 9) {
      throw new Error('Unable to serialize array field mt_Ctime - length must be 9')
    }
    // Serialize message field [mt_Ctime]
    bufferOffset = _arraySerializer.int32(obj.mt_Ctime, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_Rtime] has the right length
    if (obj.mt_Rtime.length !== 9) {
      throw new Error('Unable to serialize array field mt_Rtime - length must be 9')
    }
    // Serialize message field [mt_Rtime]
    bufferOffset = _arraySerializer.int32(obj.mt_Rtime, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_sysclk] has the right length
    if (obj.mt_sysclk.length !== 9) {
      throw new Error('Unable to serialize array field mt_sysclk - length must be 9')
    }
    // Serialize message field [mt_sysclk]
    bufferOffset = _arraySerializer.int32(obj.mt_sysclk, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_smptime] has the right length
    if (obj.mt_smptime.length !== 9) {
      throw new Error('Unable to serialize array field mt_smptime - length must be 9')
    }
    // Serialize message field [mt_smptime]
    bufferOffset = _arraySerializer.int32(obj.mt_smptime, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_cputmp] has the right length
    if (obj.mt_cputmp.length !== 9) {
      throw new Error('Unable to serialize array field mt_cputmp - length must be 9')
    }
    // Serialize message field [mt_cputmp]
    bufferOffset = _arraySerializer.int32(obj.mt_cputmp, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_mttmp] has the right length
    if (obj.mt_mttmp.length !== 9) {
      throw new Error('Unable to serialize array field mt_mttmp - length must be 9')
    }
    // Serialize message field [mt_mttmp]
    bufferOffset = _arraySerializer.int32(obj.mt_mttmp, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_invlt] has the right length
    if (obj.mt_invlt.length !== 9) {
      throw new Error('Unable to serialize array field mt_invlt - length must be 9')
    }
    // Serialize message field [mt_invlt]
    bufferOffset = _arraySerializer.int32(obj.mt_invlt, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_incrt] has the right length
    if (obj.mt_incrt.length !== 9) {
      throw new Error('Unable to serialize array field mt_incrt - length must be 9')
    }
    // Serialize message field [mt_incrt]
    bufferOffset = _arraySerializer.int32(obj.mt_incrt, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_PWMfrq] has the right length
    if (obj.mt_PWMfrq.length !== 9) {
      throw new Error('Unable to serialize array field mt_PWMfrq - length must be 9')
    }
    // Serialize message field [mt_PWMfrq]
    bufferOffset = _arraySerializer.int32(obj.mt_PWMfrq, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_ecd] has the right length
    if (obj.mt_ecd.length !== 9) {
      throw new Error('Unable to serialize array field mt_ecd - length must be 9')
    }
    // Serialize message field [mt_ecd]
    bufferOffset = _arraySerializer.int32(obj.mt_ecd, buffer, bufferOffset, 9);
    // Check that the constant length array field [mt_ecdcnt] has the right length
    if (obj.mt_ecdcnt.length !== 9) {
      throw new Error('Unable to serialize array field mt_ecdcnt - length must be 9')
    }
    // Serialize message field [mt_ecdcnt]
    bufferOffset = _arraySerializer.int32(obj.mt_ecdcnt, buffer, bufferOffset, 9);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Robot_fdstatus
    let len;
    let data = new Robot_fdstatus(null);
    // Deserialize message field [mt_mode]
    data.mt_mode = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_Gpst]
    data.mt_Gpst = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_Cpst]
    data.mt_Cpst = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_Lpst]
    data.mt_Lpst = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_Gspd]
    data.mt_Gspd = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_Cspd]
    data.mt_Cspd = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_Lspd]
    data.mt_Lspd = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_Gtq]
    data.mt_Gtq = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_PWMduty]
    data.mt_PWMduty = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_Gtime]
    data.mt_Gtime = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_Ctime]
    data.mt_Ctime = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_Rtime]
    data.mt_Rtime = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_sysclk]
    data.mt_sysclk = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_smptime]
    data.mt_smptime = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_cputmp]
    data.mt_cputmp = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_mttmp]
    data.mt_mttmp = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_invlt]
    data.mt_invlt = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_incrt]
    data.mt_incrt = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_PWMfrq]
    data.mt_PWMfrq = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_ecd]
    data.mt_ecd = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    // Deserialize message field [mt_ecdcnt]
    data.mt_ecdcnt = _arrayDeserializer.int32(buffer, bufferOffset, 9)
    return data;
  }

  static getMessageSize(object) {
    return 756;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bmirobot_msg/Robot_fdstatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '96575b502188e773da8e23f3a4f7ae68';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[9] mt_mode
    int32[9] mt_Gpst
    int32[9] mt_Cpst
    int32[9] mt_Lpst
    int32[9] mt_Gspd
    int32[9] mt_Cspd
    int32[9] mt_Lspd
    int32[9] mt_Gtq
    int32[9] mt_PWMduty
    int32[9] mt_Gtime
    int32[9] mt_Ctime
    int32[9] mt_Rtime
    int32[9] mt_sysclk
    int32[9] mt_smptime
    int32[9] mt_cputmp
    int32[9] mt_mttmp
    int32[9] mt_invlt
    int32[9] mt_incrt
    int32[9] mt_PWMfrq
    int32[9] mt_ecd
    int32[9] mt_ecdcnt
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Robot_fdstatus(null);
    if (msg.mt_mode !== undefined) {
      resolved.mt_mode = msg.mt_mode;
    }
    else {
      resolved.mt_mode = new Array(9).fill(0)
    }

    if (msg.mt_Gpst !== undefined) {
      resolved.mt_Gpst = msg.mt_Gpst;
    }
    else {
      resolved.mt_Gpst = new Array(9).fill(0)
    }

    if (msg.mt_Cpst !== undefined) {
      resolved.mt_Cpst = msg.mt_Cpst;
    }
    else {
      resolved.mt_Cpst = new Array(9).fill(0)
    }

    if (msg.mt_Lpst !== undefined) {
      resolved.mt_Lpst = msg.mt_Lpst;
    }
    else {
      resolved.mt_Lpst = new Array(9).fill(0)
    }

    if (msg.mt_Gspd !== undefined) {
      resolved.mt_Gspd = msg.mt_Gspd;
    }
    else {
      resolved.mt_Gspd = new Array(9).fill(0)
    }

    if (msg.mt_Cspd !== undefined) {
      resolved.mt_Cspd = msg.mt_Cspd;
    }
    else {
      resolved.mt_Cspd = new Array(9).fill(0)
    }

    if (msg.mt_Lspd !== undefined) {
      resolved.mt_Lspd = msg.mt_Lspd;
    }
    else {
      resolved.mt_Lspd = new Array(9).fill(0)
    }

    if (msg.mt_Gtq !== undefined) {
      resolved.mt_Gtq = msg.mt_Gtq;
    }
    else {
      resolved.mt_Gtq = new Array(9).fill(0)
    }

    if (msg.mt_PWMduty !== undefined) {
      resolved.mt_PWMduty = msg.mt_PWMduty;
    }
    else {
      resolved.mt_PWMduty = new Array(9).fill(0)
    }

    if (msg.mt_Gtime !== undefined) {
      resolved.mt_Gtime = msg.mt_Gtime;
    }
    else {
      resolved.mt_Gtime = new Array(9).fill(0)
    }

    if (msg.mt_Ctime !== undefined) {
      resolved.mt_Ctime = msg.mt_Ctime;
    }
    else {
      resolved.mt_Ctime = new Array(9).fill(0)
    }

    if (msg.mt_Rtime !== undefined) {
      resolved.mt_Rtime = msg.mt_Rtime;
    }
    else {
      resolved.mt_Rtime = new Array(9).fill(0)
    }

    if (msg.mt_sysclk !== undefined) {
      resolved.mt_sysclk = msg.mt_sysclk;
    }
    else {
      resolved.mt_sysclk = new Array(9).fill(0)
    }

    if (msg.mt_smptime !== undefined) {
      resolved.mt_smptime = msg.mt_smptime;
    }
    else {
      resolved.mt_smptime = new Array(9).fill(0)
    }

    if (msg.mt_cputmp !== undefined) {
      resolved.mt_cputmp = msg.mt_cputmp;
    }
    else {
      resolved.mt_cputmp = new Array(9).fill(0)
    }

    if (msg.mt_mttmp !== undefined) {
      resolved.mt_mttmp = msg.mt_mttmp;
    }
    else {
      resolved.mt_mttmp = new Array(9).fill(0)
    }

    if (msg.mt_invlt !== undefined) {
      resolved.mt_invlt = msg.mt_invlt;
    }
    else {
      resolved.mt_invlt = new Array(9).fill(0)
    }

    if (msg.mt_incrt !== undefined) {
      resolved.mt_incrt = msg.mt_incrt;
    }
    else {
      resolved.mt_incrt = new Array(9).fill(0)
    }

    if (msg.mt_PWMfrq !== undefined) {
      resolved.mt_PWMfrq = msg.mt_PWMfrq;
    }
    else {
      resolved.mt_PWMfrq = new Array(9).fill(0)
    }

    if (msg.mt_ecd !== undefined) {
      resolved.mt_ecd = msg.mt_ecd;
    }
    else {
      resolved.mt_ecd = new Array(9).fill(0)
    }

    if (msg.mt_ecdcnt !== undefined) {
      resolved.mt_ecdcnt = msg.mt_ecdcnt;
    }
    else {
      resolved.mt_ecdcnt = new Array(9).fill(0)
    }

    return resolved;
    }
};

module.exports = Robot_fdstatus;
