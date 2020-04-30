// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: pose.proto

#ifndef PROTOBUF_pose_2eproto__INCLUDED
#define PROTOBUF_pose_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
#include "vector3d.pb.h"
#include "quaternion.pb.h"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_pose_2eproto();
void protobuf_AssignDesc_pose_2eproto();
void protobuf_ShutdownFile_pose_2eproto();

class Pose;

// ===================================================================

class Pose : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Pose) */ {
 public:
  Pose();
  virtual ~Pose();

  Pose(const Pose& from);

  inline Pose& operator=(const Pose& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Pose& default_instance();

  void Swap(Pose* other);

  // implements Message ----------------------------------------------

  inline Pose* New() const { return New(NULL); }

  Pose* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Pose& from);
  void MergeFrom(const Pose& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(Pose* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional string name = 1;
  bool has_name() const;
  void clear_name();
  static const int kNameFieldNumber = 1;
  const ::std::string& name() const;
  void set_name(const ::std::string& value);
  void set_name(const char* value);
  void set_name(const char* value, size_t size);
  ::std::string* mutable_name();
  ::std::string* release_name();
  void set_allocated_name(::std::string* name);

  // optional uint32 id = 2;
  bool has_id() const;
  void clear_id();
  static const int kIdFieldNumber = 2;
  ::google::protobuf::uint32 id() const;
  void set_id(::google::protobuf::uint32 value);

  // required .gazebo.msgs.Vector3d position = 3;
  bool has_position() const;
  void clear_position();
  static const int kPositionFieldNumber = 3;
  const ::gazebo::msgs::Vector3d& position() const;
  ::gazebo::msgs::Vector3d* mutable_position();
  ::gazebo::msgs::Vector3d* release_position();
  void set_allocated_position(::gazebo::msgs::Vector3d* position);

  // required .gazebo.msgs.Quaternion orientation = 4;
  bool has_orientation() const;
  void clear_orientation();
  static const int kOrientationFieldNumber = 4;
  const ::gazebo::msgs::Quaternion& orientation() const;
  ::gazebo::msgs::Quaternion* mutable_orientation();
  ::gazebo::msgs::Quaternion* release_orientation();
  void set_allocated_orientation(::gazebo::msgs::Quaternion* orientation);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Pose)
 private:
  inline void set_has_name();
  inline void clear_has_name();
  inline void set_has_id();
  inline void clear_has_id();
  inline void set_has_position();
  inline void clear_has_position();
  inline void set_has_orientation();
  inline void clear_has_orientation();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr name_;
  ::gazebo::msgs::Vector3d* position_;
  ::gazebo::msgs::Quaternion* orientation_;
  ::google::protobuf::uint32 id_;
  friend void  protobuf_AddDesc_pose_2eproto();
  friend void protobuf_AssignDesc_pose_2eproto();
  friend void protobuf_ShutdownFile_pose_2eproto();

  void InitAsDefaultInstance();
  static Pose* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Pose

// optional string name = 1;
inline bool Pose::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Pose::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Pose::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Pose::clear_name() {
  name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_name();
}
inline const ::std::string& Pose::name() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Pose.name)
  return name_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Pose::set_name(const ::std::string& value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Pose.name)
}
inline void Pose::set_name(const char* value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Pose.name)
}
inline void Pose::set_name(const char* value, size_t size) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Pose.name)
}
inline ::std::string* Pose::mutable_name() {
  set_has_name();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Pose.name)
  return name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Pose::release_name() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Pose.name)
  clear_has_name();
  return name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Pose::set_allocated_name(::std::string* name) {
  if (name != NULL) {
    set_has_name();
  } else {
    clear_has_name();
  }
  name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), name);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Pose.name)
}

// optional uint32 id = 2;
inline bool Pose::has_id() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Pose::set_has_id() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Pose::clear_has_id() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Pose::clear_id() {
  id_ = 0u;
  clear_has_id();
}
inline ::google::protobuf::uint32 Pose::id() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Pose.id)
  return id_;
}
inline void Pose::set_id(::google::protobuf::uint32 value) {
  set_has_id();
  id_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Pose.id)
}

// required .gazebo.msgs.Vector3d position = 3;
inline bool Pose::has_position() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Pose::set_has_position() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Pose::clear_has_position() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Pose::clear_position() {
  if (position_ != NULL) position_->::gazebo::msgs::Vector3d::Clear();
  clear_has_position();
}
inline const ::gazebo::msgs::Vector3d& Pose::position() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Pose.position)
  return position_ != NULL ? *position_ : *default_instance_->position_;
}
inline ::gazebo::msgs::Vector3d* Pose::mutable_position() {
  set_has_position();
  if (position_ == NULL) {
    position_ = new ::gazebo::msgs::Vector3d;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Pose.position)
  return position_;
}
inline ::gazebo::msgs::Vector3d* Pose::release_position() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Pose.position)
  clear_has_position();
  ::gazebo::msgs::Vector3d* temp = position_;
  position_ = NULL;
  return temp;
}
inline void Pose::set_allocated_position(::gazebo::msgs::Vector3d* position) {
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
inline bool Pose::has_orientation() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Pose::set_has_orientation() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Pose::clear_has_orientation() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Pose::clear_orientation() {
  if (orientation_ != NULL) orientation_->::gazebo::msgs::Quaternion::Clear();
  clear_has_orientation();
}
inline const ::gazebo::msgs::Quaternion& Pose::orientation() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Pose.orientation)
  return orientation_ != NULL ? *orientation_ : *default_instance_->orientation_;
}
inline ::gazebo::msgs::Quaternion* Pose::mutable_orientation() {
  set_has_orientation();
  if (orientation_ == NULL) {
    orientation_ = new ::gazebo::msgs::Quaternion;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Pose.orientation)
  return orientation_;
}
inline ::gazebo::msgs::Quaternion* Pose::release_orientation() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Pose.orientation)
  clear_has_orientation();
  ::gazebo::msgs::Quaternion* temp = orientation_;
  orientation_ = NULL;
  return temp;
}
inline void Pose::set_allocated_orientation(::gazebo::msgs::Quaternion* orientation) {
  delete orientation_;
  orientation_ = orientation;
  if (orientation) {
    set_has_orientation();
  } else {
    clear_has_orientation();
  }
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Pose.orientation)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_pose_2eproto__INCLUDED
