// Auto-generated. Do not edit!

// (in-package Car_Spraying.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SetTwoAngle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id0 = null;
      this.angle0 = null;
      this.id1 = null;
      this.angle1 = null;
    }
    else {
      if (initObj.hasOwnProperty('id0')) {
        this.id0 = initObj.id0
      }
      else {
        this.id0 = 0;
      }
      if (initObj.hasOwnProperty('angle0')) {
        this.angle0 = initObj.angle0
      }
      else {
        this.angle0 = 0.0;
      }
      if (initObj.hasOwnProperty('id1')) {
        this.id1 = initObj.id1
      }
      else {
        this.id1 = 0;
      }
      if (initObj.hasOwnProperty('angle1')) {
        this.angle1 = initObj.angle1
      }
      else {
        this.angle1 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetTwoAngle
    // Serialize message field [id0]
    bufferOffset = _serializer.uint8(obj.id0, buffer, bufferOffset);
    // Serialize message field [angle0]
    bufferOffset = _serializer.float32(obj.angle0, buffer, bufferOffset);
    // Serialize message field [id1]
    bufferOffset = _serializer.uint8(obj.id1, buffer, bufferOffset);
    // Serialize message field [angle1]
    bufferOffset = _serializer.float32(obj.angle1, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTwoAngle
    let len;
    let data = new SetTwoAngle(null);
    // Deserialize message field [id0]
    data.id0 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [angle0]
    data.angle0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [id1]
    data.id1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [angle1]
    data.angle1 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'Car_Spraying/SetTwoAngle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f1d10f92f012da8da765573aa414a014';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 id0
    float32 angle0
    uint8 id1
    float32 angle1
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetTwoAngle(null);
    if (msg.id0 !== undefined) {
      resolved.id0 = msg.id0;
    }
    else {
      resolved.id0 = 0
    }

    if (msg.angle0 !== undefined) {
      resolved.angle0 = msg.angle0;
    }
    else {
      resolved.angle0 = 0.0
    }

    if (msg.id1 !== undefined) {
      resolved.id1 = msg.id1;
    }
    else {
      resolved.id1 = 0
    }

    if (msg.angle1 !== undefined) {
      resolved.angle1 = msg.angle1;
    }
    else {
      resolved.angle1 = 0.0
    }

    return resolved;
    }
};

module.exports = SetTwoAngle;
