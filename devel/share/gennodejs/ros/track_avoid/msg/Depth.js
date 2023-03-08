// Auto-generated. Do not edit!

// (in-package track_avoid.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Depth {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.locate = null;
      this.depth = null;
    }
    else {
      if (initObj.hasOwnProperty('locate')) {
        this.locate = initObj.locate
      }
      else {
        this.locate = 0;
      }
      if (initObj.hasOwnProperty('depth')) {
        this.depth = initObj.depth
      }
      else {
        this.depth = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Depth
    // Serialize message field [locate]
    bufferOffset = _serializer.uint16(obj.locate, buffer, bufferOffset);
    // Serialize message field [depth]
    bufferOffset = _serializer.uint16(obj.depth, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Depth
    let len;
    let data = new Depth(null);
    // Deserialize message field [locate]
    data.locate = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [depth]
    data.depth = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'track_avoid/Depth';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b0af3060455b6b7cf46be4171b872aa5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16 locate
    uint16 depth
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Depth(null);
    if (msg.locate !== undefined) {
      resolved.locate = msg.locate;
    }
    else {
      resolved.locate = 0
    }

    if (msg.depth !== undefined) {
      resolved.depth = msg.depth;
    }
    else {
      resolved.depth = 0
    }

    return resolved;
    }
};

module.exports = Depth;
