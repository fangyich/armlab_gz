// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: pose.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "pose.pb.h"

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

namespace gazebo {
namespace msgs {

namespace {

const ::google::protobuf::Descriptor* Pose_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Pose_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_pose_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_pose_2eproto() {
  protobuf_AddDesc_pose_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "pose.proto");
  GOOGLE_CHECK(file != NULL);
  Pose_descriptor_ = file->message_type(0);
  static const int Pose_offsets_[4] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Pose, name_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Pose, id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Pose, position_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Pose, orientation_),
  };
  Pose_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      Pose_descriptor_,
      Pose::default_instance_,
      Pose_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Pose, _has_bits_[0]),
      -1,
      -1,
      sizeof(Pose),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Pose, _internal_metadata_),
      -1);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_pose_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      Pose_descriptor_, &Pose::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_pose_2eproto() {
  delete Pose::default_instance_;
  delete Pose_reflection_;
}

void protobuf_AddDesc_pose_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_pose_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::gazebo::msgs::protobuf_AddDesc_vector3d_2eproto();
  ::gazebo::msgs::protobuf_AddDesc_quaternion_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\npose.proto\022\013gazebo.msgs\032\016vector3d.prot"
    "o\032\020quaternion.proto\"w\n\004Pose\022\014\n\004name\030\001 \001("
    "\t\022\n\n\002id\030\002 \001(\r\022\'\n\010position\030\003 \002(\0132\025.gazebo"
    ".msgs.Vector3d\022,\n\013orientation\030\004 \002(\0132\027.ga"
    "zebo.msgs.Quaternion", 180);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "pose.proto", &protobuf_RegisterTypes);
  Pose::default_instance_ = new Pose();
  Pose::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_pose_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_pose_2eproto {
  StaticDescriptorInitializer_pose_2eproto() {
    protobuf_AddDesc_pose_2eproto();
  }
} static_descriptor_initializer_pose_2eproto_;

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Pose::kNameFieldNumber;
const int Pose::kIdFieldNumber;
const int Pose::kPositionFieldNumber;
const int Pose::kOrientationFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Pose::Pose()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:gazebo.msgs.Pose)
}

void Pose::InitAsDefaultInstance() {
  position_ = const_cast< ::gazebo::msgs::Vector3d*>(&::gazebo::msgs::Vector3d::default_instance());
  orientation_ = const_cast< ::gazebo::msgs::Quaternion*>(&::gazebo::msgs::Quaternion::default_instance());
}

Pose::Pose(const Pose& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:gazebo.msgs.Pose)
}

void Pose::SharedCtor() {
  ::google::protobuf::internal::GetEmptyString();
  _cached_size_ = 0;
  name_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  id_ = 0u;
  position_ = NULL;
  orientation_ = NULL;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Pose::~Pose() {
  // @@protoc_insertion_point(destructor:gazebo.msgs.Pose)
  SharedDtor();
}

void Pose::SharedDtor() {
  name_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (this != default_instance_) {
    delete position_;
    delete orientation_;
  }
}

void Pose::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Pose::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Pose_descriptor_;
}

const Pose& Pose::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_pose_2eproto();
  return *default_instance_;
}

Pose* Pose::default_instance_ = NULL;

Pose* Pose::New(::google::protobuf::Arena* arena) const {
  Pose* n = new Pose;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Pose::Clear() {
// @@protoc_insertion_point(message_clear_start:gazebo.msgs.Pose)
  if (_has_bits_[0 / 32] & 15u) {
    if (has_name()) {
      name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    }
    id_ = 0u;
    if (has_position()) {
      if (position_ != NULL) position_->::gazebo::msgs::Vector3d::Clear();
    }
    if (has_orientation()) {
      if (orientation_ != NULL) orientation_->::gazebo::msgs::Quaternion::Clear();
    }
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool Pose::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:gazebo.msgs.Pose)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional string name = 1;
      case 1: {
        if (tag == 10) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_name()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->name().data(), this->name().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "gazebo.msgs.Pose.name");
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(16)) goto parse_id;
        break;
      }

      // optional uint32 id = 2;
      case 2: {
        if (tag == 16) {
         parse_id:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &id_)));
          set_has_id();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(26)) goto parse_position;
        break;
      }

      // required .gazebo.msgs.Vector3d position = 3;
      case 3: {
        if (tag == 26) {
         parse_position:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_position()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(34)) goto parse_orientation;
        break;
      }

      // required .gazebo.msgs.Quaternion orientation = 4;
      case 4: {
        if (tag == 34) {
         parse_orientation:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_orientation()));
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
  // @@protoc_insertion_point(parse_success:gazebo.msgs.Pose)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:gazebo.msgs.Pose)
  return false;
#undef DO_
}

void Pose::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:gazebo.msgs.Pose)
  // optional string name = 1;
  if (has_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->name().data(), this->name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "gazebo.msgs.Pose.name");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->name(), output);
  }

  // optional uint32 id = 2;
  if (has_id()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->id(), output);
  }

  // required .gazebo.msgs.Vector3d position = 3;
  if (has_position()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, *this->position_, output);
  }

  // required .gazebo.msgs.Quaternion orientation = 4;
  if (has_orientation()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      4, *this->orientation_, output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:gazebo.msgs.Pose)
}

::google::protobuf::uint8* Pose::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:gazebo.msgs.Pose)
  // optional string name = 1;
  if (has_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->name().data(), this->name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "gazebo.msgs.Pose.name");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->name(), target);
  }

  // optional uint32 id = 2;
  if (has_id()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->id(), target);
  }

  // required .gazebo.msgs.Vector3d position = 3;
  if (has_position()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        3, *this->position_, false, target);
  }

  // required .gazebo.msgs.Quaternion orientation = 4;
  if (has_orientation()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        4, *this->orientation_, false, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:gazebo.msgs.Pose)
  return target;
}

int Pose::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:gazebo.msgs.Pose)
  int total_size = 0;

  if (has_position()) {
    // required .gazebo.msgs.Vector3d position = 3;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->position_);
  }

  if (has_orientation()) {
    // required .gazebo.msgs.Quaternion orientation = 4;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->orientation_);
  }

  return total_size;
}
int Pose::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:gazebo.msgs.Pose)
  int total_size = 0;

  if (((_has_bits_[0] & 0x0000000c) ^ 0x0000000c) == 0) {  // All required fields are present.
    // required .gazebo.msgs.Vector3d position = 3;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->position_);

    // required .gazebo.msgs.Quaternion orientation = 4;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->orientation_);

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  if (_has_bits_[0 / 32] & 3u) {
    // optional string name = 1;
    if (has_name()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->name());
    }

    // optional uint32 id = 2;
    if (has_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->id());
    }

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

void Pose::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:gazebo.msgs.Pose)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const Pose* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const Pose>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:gazebo.msgs.Pose)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:gazebo.msgs.Pose)
    MergeFrom(*source);
  }
}

void Pose::MergeFrom(const Pose& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:gazebo.msgs.Pose)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_name()) {
      set_has_name();
      name_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.name_);
    }
    if (from.has_id()) {
      set_id(from.id());
    }
    if (from.has_position()) {
      mutable_position()->::gazebo::msgs::Vector3d::MergeFrom(from.position());
    }
    if (from.has_orientation()) {
      mutable_orientation()->::gazebo::msgs::Quaternion::MergeFrom(from.orientation());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->MergeFrom(from.unknown_fields());
  }
}

void Pose::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:gazebo.msgs.Pose)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Pose::CopyFrom(const Pose& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:gazebo.msgs.Pose)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Pose::IsInitialized() const {
  if ((_has_bits_[0] & 0x0000000c) != 0x0000000c) return false;

  if (has_position()) {
    if (!this->position_->IsInitialized()) return false;
  }
  if (has_orientation()) {
    if (!this->orientation_->IsInitialized()) return false;
  }
  return true;
}

void Pose::Swap(Pose* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Pose::InternalSwap(Pose* other) {
  name_.Swap(&other->name_);
  std::swap(id_, other->id_);
  std::swap(position_, other->position_);
  std::swap(orientation_, other->orientation_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata Pose::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Pose_descriptor_;
  metadata.reflection = Pose_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// Pose

// optional string name = 1;
bool Pose::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void Pose::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
void Pose::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
void Pose::clear_name() {
  name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_name();
}
 const ::std::string& Pose::name() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Pose.name)
  return name_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void Pose::set_name(const ::std::string& value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Pose.name)
}
 void Pose::set_name(const char* value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Pose.name)
}
 void Pose::set_name(const char* value, size_t size) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Pose.name)
}
 ::std::string* Pose::mutable_name() {
  set_has_name();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Pose.name)
  return name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 ::std::string* Pose::release_name() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Pose.name)
  clear_has_name();
  return name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void Pose::set_allocated_name(::std::string* name) {
  if (name != NULL) {
    set_has_name();
  } else {
    clear_has_name();
  }
  name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), name);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Pose.name)
}

// optional uint32 id = 2;
bool Pose::has_id() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void Pose::set_has_id() {
  _has_bits_[0] |= 0x00000002u;
}
void Pose::clear_has_id() {
  _has_bits_[0] &= ~0x00000002u;
}
void Pose::clear_id() {
  id_ = 0u;
  clear_has_id();
}
 ::google::protobuf::uint32 Pose::id() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Pose.id)
  return id_;
}
 void Pose::set_id(::google::protobuf::uint32 value) {
  set_has_id();
  id_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Pose.id)
}

// required .gazebo.msgs.Vector3d position = 3;
bool Pose::has_position() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void Pose::set_has_position() {
  _has_bits_[0] |= 0x00000004u;
}
void Pose::clear_has_position() {
  _has_bits_[0] &= ~0x00000004u;
}
void Pose::clear_position() {
  if (position_ != NULL) position_->::gazebo::msgs::Vector3d::Clear();
  clear_has_position();
}
const ::gazebo::msgs::Vector3d& Pose::position() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Pose.position)
  return position_ != NULL ? *position_ : *default_instance_->position_;
}
::gazebo::msgs::Vector3d* Pose::mutable_position() {
  set_has_position();
  if (position_ == NULL) {
    position_ = new ::gazebo::msgs::Vector3d;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Pose.position)
  return position_;
}
::gazebo::msgs::Vector3d* Pose::release_position() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Pose.position)
  clear_has_position();
  ::gazebo::msgs::Vector3d* temp = position_;
  position_ = NULL;
  return temp;
}
void Pose::set_allocated_position(::gazebo::msgs::Vector3d* position) {
  delete position_;
  position_ = position;
  if (position) {
    set_has_position();
  } else {
    clear_has_position();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Pose.position)
}

// required .gazebo.msgs.Quaternion orientation = 4;
bool Pose::has_orientation() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
void Pose::set_has_orientation() {
  _has_bits_[0] |= 0x00000008u;
}
void Pose::clear_has_orientation() {
  _has_bits_[0] &= ~0x00000008u;
}
void Pose::clear_orientation() {
  if (orientation_ != NULL) orientation_->::gazebo::msgs::Quaternion::Clear();
  clear_has_orientation();
}
const ::gazebo::msgs::Quaternion& Pose::orientation() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Pose.orientation)
  return orientation_ != NULL ? *orientation_ : *default_instance_->orientation_;
}
::gazebo::msgs::Quaternion* Pose::mutable_orientation() {
  set_has_orientation();
  if (orientation_ == NULL) {
    orientation_ = new ::gazebo::msgs::Quaternion;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Pose.orientation)
  return orientation_;
}
::gazebo::msgs::Quaternion* Pose::release_orientation() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Pose.orientation)
  clear_has_orientation();
  ::gazebo::msgs::Quaternion* temp = orientation_;
  orientation_ = NULL;
  return temp;
}
void Pose::set_allocated_orientation(::gazebo::msgs::Quaternion* orientation) {
  delete orientation_;
  orientation_ = orientation;
  if (orientation) {
    set_has_orientation();
  } else {
    clear_has_orientation();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Pose.orientation)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

// @@protoc_insertion_point(global_scope)