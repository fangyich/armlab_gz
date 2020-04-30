// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: rexarm_poses.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "rexarm_poses.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace rexarm_poses_msgs {
namespace msgs {

namespace {

const ::google::protobuf::Descriptor* RexarmPoses_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  RexarmPoses_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_rexarm_5fposes_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_rexarm_5fposes_2eproto() {
  protobuf_AddDesc_rexarm_5fposes_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "rexarm_poses.proto");
  GOOGLE_CHECK(file != NULL);
  RexarmPoses_descriptor_ = file->message_type(0);
  static const int RexarmPoses_offsets_[5] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RexarmPoses, end_effector_pose_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RexarmPoses, base_joint_pos_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RexarmPoses, shoulder_joint_pos_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RexarmPoses, elbow_joint_pos_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RexarmPoses, wrist_joint_pos_),
  };
  RexarmPoses_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      RexarmPoses_descriptor_,
      RexarmPoses::default_instance_,
      RexarmPoses_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RexarmPoses, _has_bits_[0]),
      -1,
      -1,
      sizeof(RexarmPoses),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RexarmPoses, _internal_metadata_),
      -1);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_rexarm_5fposes_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      RexarmPoses_descriptor_, &RexarmPoses::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_rexarm_5fposes_2eproto() {
  delete RexarmPoses::default_instance_;
  delete RexarmPoses_reflection_;
}

void protobuf_AddDesc_rexarm_5fposes_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_rexarm_5fposes_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::gazebo::msgs::protobuf_AddDesc_pose_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\022rexarm_poses.proto\022\026rexarm_poses_msgs."
    "msgs\032\npose.proto\"\241\001\n\013RexarmPoses\022,\n\021end_"
    "effector_pose\030\001 \001(\0132\021.gazebo.msgs.Pose\022\026"
    "\n\016base_joint_pos\030\002 \002(\001\022\032\n\022shoulder_joint"
    "_pos\030\003 \002(\001\022\027\n\017elbow_joint_pos\030\004 \002(\001\022\027\n\017w"
    "rist_joint_pos\030\005 \002(\001", 220);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "rexarm_poses.proto", &protobuf_RegisterTypes);
  RexarmPoses::default_instance_ = new RexarmPoses();
  RexarmPoses::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_rexarm_5fposes_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_rexarm_5fposes_2eproto {
  StaticDescriptorInitializer_rexarm_5fposes_2eproto() {
    protobuf_AddDesc_rexarm_5fposes_2eproto();
  }
} static_descriptor_initializer_rexarm_5fposes_2eproto_;

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int RexarmPoses::kEndEffectorPoseFieldNumber;
const int RexarmPoses::kBaseJointPosFieldNumber;
const int RexarmPoses::kShoulderJointPosFieldNumber;
const int RexarmPoses::kElbowJointPosFieldNumber;
const int RexarmPoses::kWristJointPosFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

RexarmPoses::RexarmPoses()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:rexarm_poses_msgs.msgs.RexarmPoses)
}

void RexarmPoses::InitAsDefaultInstance() {
  end_effector_pose_ = const_cast< ::gazebo::msgs::Pose*>(&::gazebo::msgs::Pose::default_instance());
}

RexarmPoses::RexarmPoses(const RexarmPoses& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:rexarm_poses_msgs.msgs.RexarmPoses)
}

void RexarmPoses::SharedCtor() {
  _cached_size_ = 0;
  end_effector_pose_ = NULL;
  base_joint_pos_ = 0;
  shoulder_joint_pos_ = 0;
  elbow_joint_pos_ = 0;
  wrist_joint_pos_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

RexarmPoses::~RexarmPoses() {
  // @@protoc_insertion_point(destructor:rexarm_poses_msgs.msgs.RexarmPoses)
  SharedDtor();
}

void RexarmPoses::SharedDtor() {
  if (this != default_instance_) {
    delete end_effector_pose_;
  }
}

void RexarmPoses::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* RexarmPoses::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return RexarmPoses_descriptor_;
}

const RexarmPoses& RexarmPoses::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_rexarm_5fposes_2eproto();
  return *default_instance_;
}

RexarmPoses* RexarmPoses::default_instance_ = NULL;

RexarmPoses* RexarmPoses::New(::google::protobuf::Arena* arena) const {
  RexarmPoses* n = new RexarmPoses;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void RexarmPoses::Clear() {
// @@protoc_insertion_point(message_clear_start:rexarm_poses_msgs.msgs.RexarmPoses)
#if defined(__clang__)
#define ZR_HELPER_(f) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"") \
  __builtin_offsetof(RexarmPoses, f) \
  _Pragma("clang diagnostic pop")
#else
#define ZR_HELPER_(f) reinterpret_cast<char*>(\
  &reinterpret_cast<RexarmPoses*>(16)->f)
#endif

#define ZR_(first, last) do {\
  ::memset(&first, 0,\
           ZR_HELPER_(last) - ZR_HELPER_(first) + sizeof(last));\
} while (0)

  if (_has_bits_[0 / 32] & 31u) {
    ZR_(base_joint_pos_, wrist_joint_pos_);
    if (has_end_effector_pose()) {
      if (end_effector_pose_ != NULL) end_effector_pose_->::gazebo::msgs::Pose::Clear();
    }
  }

#undef ZR_HELPER_
#undef ZR_

  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool RexarmPoses::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:rexarm_poses_msgs.msgs.RexarmPoses)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .gazebo.msgs.Pose end_effector_pose = 1;
      case 1: {
        if (tag == 10) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_end_effector_pose()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_base_joint_pos;
        break;
      }

      // required double base_joint_pos = 2;
      case 2: {
        if (tag == 17) {
         parse_base_joint_pos:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &base_joint_pos_)));
          set_has_base_joint_pos();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(25)) goto parse_shoulder_joint_pos;
        break;
      }

      // required double shoulder_joint_pos = 3;
      case 3: {
        if (tag == 25) {
         parse_shoulder_joint_pos:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &shoulder_joint_pos_)));
          set_has_shoulder_joint_pos();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(33)) goto parse_elbow_joint_pos;
        break;
      }

      // required double elbow_joint_pos = 4;
      case 4: {
        if (tag == 33) {
         parse_elbow_joint_pos:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &elbow_joint_pos_)));
          set_has_elbow_joint_pos();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(41)) goto parse_wrist_joint_pos;
        break;
      }

      // required double wrist_joint_pos = 5;
      case 5: {
        if (tag == 41) {
         parse_wrist_joint_pos:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &wrist_joint_pos_)));
          set_has_wrist_joint_pos();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:rexarm_poses_msgs.msgs.RexarmPoses)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:rexarm_poses_msgs.msgs.RexarmPoses)
  return false;
#undef DO_
}

void RexarmPoses::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:rexarm_poses_msgs.msgs.RexarmPoses)
  // optional .gazebo.msgs.Pose end_effector_pose = 1;
  if (has_end_effector_pose()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, *this->end_effector_pose_, output);
  }

  // required double base_joint_pos = 2;
  if (has_base_joint_pos()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->base_joint_pos(), output);
  }

  // required double shoulder_joint_pos = 3;
  if (has_shoulder_joint_pos()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->shoulder_joint_pos(), output);
  }

  // required double elbow_joint_pos = 4;
  if (has_elbow_joint_pos()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->elbow_joint_pos(), output);
  }

  // required double wrist_joint_pos = 5;
  if (has_wrist_joint_pos()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(5, this->wrist_joint_pos(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:rexarm_poses_msgs.msgs.RexarmPoses)
}

::google::protobuf::uint8* RexarmPoses::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:rexarm_poses_msgs.msgs.RexarmPoses)
  // optional .gazebo.msgs.Pose end_effector_pose = 1;
  if (has_end_effector_pose()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        1, *this->end_effector_pose_, false, target);
  }

  // required double base_joint_pos = 2;
  if (has_base_joint_pos()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->base_joint_pos(), target);
  }

  // required double shoulder_joint_pos = 3;
  if (has_shoulder_joint_pos()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->shoulder_joint_pos(), target);
  }

  // required double elbow_joint_pos = 4;
  if (has_elbow_joint_pos()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->elbow_joint_pos(), target);
  }

  // required double wrist_joint_pos = 5;
  if (has_wrist_joint_pos()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(5, this->wrist_joint_pos(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:rexarm_poses_msgs.msgs.RexarmPoses)
  return target;
}

int RexarmPoses::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:rexarm_poses_msgs.msgs.RexarmPoses)
  int total_size = 0;

  if (has_base_joint_pos()) {
    // required double base_joint_pos = 2;
    total_size += 1 + 8;
  }

  if (has_shoulder_joint_pos()) {
    // required double shoulder_joint_pos = 3;
    total_size += 1 + 8;
  }

  if (has_elbow_joint_pos()) {
    // required double elbow_joint_pos = 4;
    total_size += 1 + 8;
  }

  if (has_wrist_joint_pos()) {
    // required double wrist_joint_pos = 5;
    total_size += 1 + 8;
  }

  return total_size;
}
int RexarmPoses::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:rexarm_poses_msgs.msgs.RexarmPoses)
  int total_size = 0;

  if (((_has_bits_[0] & 0x0000001e) ^ 0x0000001e) == 0) {  // All required fields are present.
    // required double base_joint_pos = 2;
    total_size += 1 + 8;

    // required double shoulder_joint_pos = 3;
    total_size += 1 + 8;

    // required double elbow_joint_pos = 4;
    total_size += 1 + 8;

    // required double wrist_joint_pos = 5;
    total_size += 1 + 8;

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  // optional .gazebo.msgs.Pose end_effector_pose = 1;
  if (has_end_effector_pose()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->end_effector_pose_);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void RexarmPoses::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:rexarm_poses_msgs.msgs.RexarmPoses)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const RexarmPoses* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const RexarmPoses>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:rexarm_poses_msgs.msgs.RexarmPoses)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:rexarm_poses_msgs.msgs.RexarmPoses)
    MergeFrom(*source);
  }
}

void RexarmPoses::MergeFrom(const RexarmPoses& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:rexarm_poses_msgs.msgs.RexarmPoses)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_end_effector_pose()) {
      mutable_end_effector_pose()->::gazebo::msgs::Pose::MergeFrom(from.end_effector_pose());
    }
    if (from.has_base_joint_pos()) {
      set_base_joint_pos(from.base_joint_pos());
    }
    if (from.has_shoulder_joint_pos()) {
      set_shoulder_joint_pos(from.shoulder_joint_pos());
    }
    if (from.has_elbow_joint_pos()) {
      set_elbow_joint_pos(from.elbow_joint_pos());
    }
    if (from.has_wrist_joint_pos()) {
      set_wrist_joint_pos(from.wrist_joint_pos());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->MergeFrom(from.unknown_fields());
  }
}

void RexarmPoses::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:rexarm_poses_msgs.msgs.RexarmPoses)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void RexarmPoses::CopyFrom(const RexarmPoses& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:rexarm_poses_msgs.msgs.RexarmPoses)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RexarmPoses::IsInitialized() const {
  if ((_has_bits_[0] & 0x0000001e) != 0x0000001e) return false;

  if (has_end_effector_pose()) {
    if (!this->end_effector_pose_->IsInitialized()) return false;
  }
  return true;
}

void RexarmPoses::Swap(RexarmPoses* other) {
  if (other == this) return;
  InternalSwap(other);
}
void RexarmPoses::InternalSwap(RexarmPoses* other) {
  std::swap(end_effector_pose_, other->end_effector_pose_);
  std::swap(base_joint_pos_, other->base_joint_pos_);
  std::swap(shoulder_joint_pos_, other->shoulder_joint_pos_);
  std::swap(elbow_joint_pos_, other->elbow_joint_pos_);
  std::swap(wrist_joint_pos_, other->wrist_joint_pos_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata RexarmPoses::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = RexarmPoses_descriptor_;
  metadata.reflection = RexarmPoses_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// RexarmPoses

// optional .gazebo.msgs.Pose end_effector_pose = 1;
bool RexarmPoses::has_end_effector_pose() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void RexarmPoses::set_has_end_effector_pose() {
  _has_bits_[0] |= 0x00000001u;
}
void RexarmPoses::clear_has_end_effector_pose() {
  _has_bits_[0] &= ~0x00000001u;
}
void RexarmPoses::clear_end_effector_pose() {
  if (end_effector_pose_ != NULL) end_effector_pose_->::gazebo::msgs::Pose::Clear();
  clear_has_end_effector_pose();
}
const ::gazebo::msgs::Pose& RexarmPoses::end_effector_pose() const {
  // @@protoc_insertion_point(field_get:rexarm_poses_msgs.msgs.RexarmPoses.end_effector_pose)
  return end_effector_pose_ != NULL ? *end_effector_pose_ : *default_instance_->end_effector_pose_;
}
::gazebo::msgs::Pose* RexarmPoses::mutable_end_effector_pose() {
  set_has_end_effector_pose();
  if (end_effector_pose_ == NULL) {
    end_effector_pose_ = new ::gazebo::msgs::Pose;
  }
  // @@protoc_insertion_point(field_mutable:rexarm_poses_msgs.msgs.RexarmPoses.end_effector_pose)
  return end_effector_pose_;
}
::gazebo::msgs::Pose* RexarmPoses::release_end_effector_pose() {
  // @@protoc_insertion_point(field_release:rexarm_poses_msgs.msgs.RexarmPoses.end_effector_pose)
  clear_has_end_effector_pose();
  ::gazebo::msgs::Pose* temp = end_effector_pose_;
  end_effector_pose_ = NULL;
  return temp;
}
void RexarmPoses::set_allocated_end_effector_pose(::gazebo::msgs::Pose* end_effector_pose) {
  delete end_effector_pose_;
  end_effector_pose_ = end_effector_pose;
  if (end_effector_pose) {
    set_has_end_effector_pose();
  } else {
    clear_has_end_effector_pose();
  }
  // @@protoc_insertion_point(field_set_allocated:rexarm_poses_msgs.msgs.RexarmPoses.end_effector_pose)
}

// required double base_joint_pos = 2;
bool RexarmPoses::has_base_joint_pos() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void RexarmPoses::set_has_base_joint_pos() {
  _has_bits_[0] |= 0x00000002u;
}
void RexarmPoses::clear_has_base_joint_pos() {
  _has_bits_[0] &= ~0x00000002u;
}
void RexarmPoses::clear_base_joint_pos() {
  base_joint_pos_ = 0;
  clear_has_base_joint_pos();
}
 double RexarmPoses::base_joint_pos() const {
  // @@protoc_insertion_point(field_get:rexarm_poses_msgs.msgs.RexarmPoses.base_joint_pos)
  return base_joint_pos_;
}
 void RexarmPoses::set_base_joint_pos(double value) {
  set_has_base_joint_pos();
  base_joint_pos_ = value;
  // @@protoc_insertion_point(field_set:rexarm_poses_msgs.msgs.RexarmPoses.base_joint_pos)
}

// required double shoulder_joint_pos = 3;
bool RexarmPoses::has_shoulder_joint_pos() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void RexarmPoses::set_has_shoulder_joint_pos() {
  _has_bits_[0] |= 0x00000004u;
}
void RexarmPoses::clear_has_shoulder_joint_pos() {
  _has_bits_[0] &= ~0x00000004u;
}
void RexarmPoses::clear_shoulder_joint_pos() {
  shoulder_joint_pos_ = 0;
  clear_has_shoulder_joint_pos();
}
 double RexarmPoses::shoulder_joint_pos() const {
  // @@protoc_insertion_point(field_get:rexarm_poses_msgs.msgs.RexarmPoses.shoulder_joint_pos)
  return shoulder_joint_pos_;
}
 void RexarmPoses::set_shoulder_joint_pos(double value) {
  set_has_shoulder_joint_pos();
  shoulder_joint_pos_ = value;
  // @@protoc_insertion_point(field_set:rexarm_poses_msgs.msgs.RexarmPoses.shoulder_joint_pos)
}

// required double elbow_joint_pos = 4;
bool RexarmPoses::has_elbow_joint_pos() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
void RexarmPoses::set_has_elbow_joint_pos() {
  _has_bits_[0] |= 0x00000008u;
}
void RexarmPoses::clear_has_elbow_joint_pos() {
  _has_bits_[0] &= ~0x00000008u;
}
void RexarmPoses::clear_elbow_joint_pos() {
  elbow_joint_pos_ = 0;
  clear_has_elbow_joint_pos();
}
 double RexarmPoses::elbow_joint_pos() const {
  // @@protoc_insertion_point(field_get:rexarm_poses_msgs.msgs.RexarmPoses.elbow_joint_pos)
  return elbow_joint_pos_;
}
 void RexarmPoses::set_elbow_joint_pos(double value) {
  set_has_elbow_joint_pos();
  elbow_joint_pos_ = value;
  // @@protoc_insertion_point(field_set:rexarm_poses_msgs.msgs.RexarmPoses.elbow_joint_pos)
}

// required double wrist_joint_pos = 5;
bool RexarmPoses::has_wrist_joint_pos() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
void RexarmPoses::set_has_wrist_joint_pos() {
  _has_bits_[0] |= 0x00000010u;
}
void RexarmPoses::clear_has_wrist_joint_pos() {
  _has_bits_[0] &= ~0x00000010u;
}
void RexarmPoses::clear_wrist_joint_pos() {
  wrist_joint_pos_ = 0;
  clear_has_wrist_joint_pos();
}
 double RexarmPoses::wrist_joint_pos() const {
  // @@protoc_insertion_point(field_get:rexarm_poses_msgs.msgs.RexarmPoses.wrist_joint_pos)
  return wrist_joint_pos_;
}
 void RexarmPoses::set_wrist_joint_pos(double value) {
  set_has_wrist_joint_pos();
  wrist_joint_pos_ = value;
  // @@protoc_insertion_point(field_set:rexarm_poses_msgs.msgs.RexarmPoses.wrist_joint_pos)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace rexarm_poses_msgs

// @@protoc_insertion_point(global_scope)