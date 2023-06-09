# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: vssref_common.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='vssref_common.proto',
  package='VSSRef',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x13vssref_common.proto\x12\x06VSSRef\"D\n\x05Robot\x12\x10\n\x08robot_id\x18\x01 \x01(\r\x12\t\n\x01x\x18\x02 \x01(\x01\x12\t\n\x01y\x18\x03 \x01(\x01\x12\x13\n\x0borientation\x18\x04 \x01(\x01\"H\n\x05\x46rame\x12 \n\tteamColor\x18\x01 \x01(\x0e\x32\r.VSSRef.Color\x12\x1d\n\x06robots\x18\x02 \x03(\x0b\x32\r.VSSRef.Robot*i\n\x04\x46oul\x12\r\n\tFREE_KICK\x10\x00\x12\x10\n\x0cPENALTY_KICK\x10\x01\x12\r\n\tGOAL_KICK\x10\x02\x12\r\n\tFREE_BALL\x10\x03\x12\x0b\n\x07KICKOFF\x10\x04\x12\x08\n\x04STOP\x10\x05\x12\x0b\n\x07GAME_ON\x10\x06*\'\n\x05\x43olor\x12\x08\n\x04\x42LUE\x10\x00\x12\n\n\x06YELLOW\x10\x01\x12\x08\n\x04NONE\x10\x02*[\n\x08Quadrant\x12\x0f\n\x0bNO_QUADRANT\x10\x00\x12\x0e\n\nQUADRANT_1\x10\x01\x12\x0e\n\nQUADRANT_2\x10\x02\x12\x0e\n\nQUADRANT_3\x10\x03\x12\x0e\n\nQUADRANT_4\x10\x04*4\n\x04Half\x12\x0b\n\x07NO_HALF\x10\x00\x12\x0e\n\nFIRST_HALF\x10\x01\x12\x0f\n\x0bSECOND_HALF\x10\x02\x62\x06proto3')
)

_FOUL = _descriptor.EnumDescriptor(
  name='Foul',
  full_name='VSSRef.Foul',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='FREE_KICK', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PENALTY_KICK', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GOAL_KICK', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FREE_BALL', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='KICKOFF', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STOP', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GAME_ON', index=6, number=6,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=175,
  serialized_end=280,
)
_sym_db.RegisterEnumDescriptor(_FOUL)

Foul = enum_type_wrapper.EnumTypeWrapper(_FOUL)
_COLOR = _descriptor.EnumDescriptor(
  name='Color',
  full_name='VSSRef.Color',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='BLUE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='YELLOW', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NONE', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=282,
  serialized_end=321,
)
_sym_db.RegisterEnumDescriptor(_COLOR)

Color = enum_type_wrapper.EnumTypeWrapper(_COLOR)
_QUADRANT = _descriptor.EnumDescriptor(
  name='Quadrant',
  full_name='VSSRef.Quadrant',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NO_QUADRANT', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='QUADRANT_1', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='QUADRANT_2', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='QUADRANT_3', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='QUADRANT_4', index=4, number=4,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=323,
  serialized_end=414,
)
_sym_db.RegisterEnumDescriptor(_QUADRANT)

Quadrant = enum_type_wrapper.EnumTypeWrapper(_QUADRANT)
_HALF = _descriptor.EnumDescriptor(
  name='Half',
  full_name='VSSRef.Half',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NO_HALF', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FIRST_HALF', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SECOND_HALF', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=416,
  serialized_end=468,
)
_sym_db.RegisterEnumDescriptor(_HALF)

Half = enum_type_wrapper.EnumTypeWrapper(_HALF)
FREE_KICK = 0
PENALTY_KICK = 1
GOAL_KICK = 2
FREE_BALL = 3
KICKOFF = 4
STOP = 5
GAME_ON = 6
BLUE = 0
YELLOW = 1
NONE = 2
NO_QUADRANT = 0
QUADRANT_1 = 1
QUADRANT_2 = 2
QUADRANT_3 = 3
QUADRANT_4 = 4
NO_HALF = 0
FIRST_HALF = 1
SECOND_HALF = 2



_ROBOT = _descriptor.Descriptor(
  name='Robot',
  full_name='VSSRef.Robot',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='robot_id', full_name='VSSRef.Robot.robot_id', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='x', full_name='VSSRef.Robot.x', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='y', full_name='VSSRef.Robot.y', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='orientation', full_name='VSSRef.Robot.orientation', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=31,
  serialized_end=99,
)


_FRAME = _descriptor.Descriptor(
  name='Frame',
  full_name='VSSRef.Frame',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='teamColor', full_name='VSSRef.Frame.teamColor', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='robots', full_name='VSSRef.Frame.robots', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=101,
  serialized_end=173,
)

_FRAME.fields_by_name['teamColor'].enum_type = _COLOR
_FRAME.fields_by_name['robots'].message_type = _ROBOT
DESCRIPTOR.message_types_by_name['Robot'] = _ROBOT
DESCRIPTOR.message_types_by_name['Frame'] = _FRAME
DESCRIPTOR.enum_types_by_name['Foul'] = _FOUL
DESCRIPTOR.enum_types_by_name['Color'] = _COLOR
DESCRIPTOR.enum_types_by_name['Quadrant'] = _QUADRANT
DESCRIPTOR.enum_types_by_name['Half'] = _HALF
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Robot = _reflection.GeneratedProtocolMessageType('Robot', (_message.Message,), dict(
  DESCRIPTOR = _ROBOT,
  __module__ = 'vssref_common_pb2'
  # @@protoc_insertion_point(class_scope:VSSRef.Robot)
  ))
_sym_db.RegisterMessage(Robot)

Frame = _reflection.GeneratedProtocolMessageType('Frame', (_message.Message,), dict(
  DESCRIPTOR = _FRAME,
  __module__ = 'vssref_common_pb2'
  # @@protoc_insertion_point(class_scope:VSSRef.Frame)
  ))
_sym_db.RegisterMessage(Frame)


# @@protoc_insertion_point(module_scope)
