// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ring_keys.proto

#ifndef PROTOBUF_ring_5fkeys_2eproto__INCLUDED
#define PROTOBUF_ring_5fkeys_2eproto__INCLUDED

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
// @@protoc_insertion_point(includes)

namespace scan_context_io {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_ring_5fkeys_2eproto();
void protobuf_AssignDesc_ring_5fkeys_2eproto();
void protobuf_ShutdownFile_ring_5fkeys_2eproto();

class RingKey;
class RingKeys;

// ===================================================================

class RingKey : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:scan_context_io.RingKey) */ {
 public:
  RingKey();
  virtual ~RingKey();

  RingKey(const RingKey& from);

  inline RingKey& operator=(const RingKey& from) {
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
  static const RingKey& default_instance();

  void Swap(RingKey* other);

  // implements Message ----------------------------------------------

  inline RingKey* New() const { return New(NULL); }

  RingKey* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const RingKey& from);
  void MergeFrom(const RingKey& from);
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
  void InternalSwap(RingKey* other);
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

  // repeated float data = 1;
  int data_size() const;
  void clear_data();
  static const int kDataFieldNumber = 1;
  float data(int index) const;
  void set_data(int index, float value);
  void add_data(float value);
  const ::google::protobuf::RepeatedField< float >&
      data() const;
  ::google::protobuf::RepeatedField< float >*
      mutable_data();

  // @@protoc_insertion_point(class_scope:scan_context_io.RingKey)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedField< float > data_;
  friend void  protobuf_AddDesc_ring_5fkeys_2eproto();
  friend void protobuf_AssignDesc_ring_5fkeys_2eproto();
  friend void protobuf_ShutdownFile_ring_5fkeys_2eproto();

  void InitAsDefaultInstance();
  static RingKey* default_instance_;
};
// -------------------------------------------------------------------

class RingKeys : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:scan_context_io.RingKeys) */ {
 public:
  RingKeys();
  virtual ~RingKeys();

  RingKeys(const RingKeys& from);

  inline RingKeys& operator=(const RingKeys& from) {
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
  static const RingKeys& default_instance();

  void Swap(RingKeys* other);

  // implements Message ----------------------------------------------

  inline RingKeys* New() const { return New(NULL); }

  RingKeys* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const RingKeys& from);
  void MergeFrom(const RingKeys& from);
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
  void InternalSwap(RingKeys* other);
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

  // repeated .scan_context_io.RingKey data = 1;
  int data_size() const;
  void clear_data();
  static const int kDataFieldNumber = 1;
  const ::scan_context_io::RingKey& data(int index) const;
  ::scan_context_io::RingKey* mutable_data(int index);
  ::scan_context_io::RingKey* add_data();
  ::google::protobuf::RepeatedPtrField< ::scan_context_io::RingKey >*
      mutable_data();
  const ::google::protobuf::RepeatedPtrField< ::scan_context_io::RingKey >&
      data() const;

  // @@protoc_insertion_point(class_scope:scan_context_io.RingKeys)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::scan_context_io::RingKey > data_;
  friend void  protobuf_AddDesc_ring_5fkeys_2eproto();
  friend void protobuf_AssignDesc_ring_5fkeys_2eproto();
  friend void protobuf_ShutdownFile_ring_5fkeys_2eproto();

  void InitAsDefaultInstance();
  static RingKeys* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// RingKey

// repeated float data = 1;
inline int RingKey::data_size() const {
  return data_.size();
}
inline void RingKey::clear_data() {
  data_.Clear();
}
inline float RingKey::data(int index) const {
  // @@protoc_insertion_point(field_get:scan_context_io.RingKey.data)
  return data_.Get(index);
}
inline void RingKey::set_data(int index, float value) {
  data_.Set(index, value);
  // @@protoc_insertion_point(field_set:scan_context_io.RingKey.data)
}
inline void RingKey::add_data(float value) {
  data_.Add(value);
  // @@protoc_insertion_point(field_add:scan_context_io.RingKey.data)
}
inline const ::google::protobuf::RepeatedField< float >&
RingKey::data() const {
  // @@protoc_insertion_point(field_list:scan_context_io.RingKey.data)
  return data_;
}
inline ::google::protobuf::RepeatedField< float >*
RingKey::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:scan_context_io.RingKey.data)
  return &data_;
}

// -------------------------------------------------------------------

// RingKeys

// repeated .scan_context_io.RingKey data = 1;
inline int RingKeys::data_size() const {
  return data_.size();
}
inline void RingKeys::clear_data() {
  data_.Clear();
}
inline const ::scan_context_io::RingKey& RingKeys::data(int index) const {
  // @@protoc_insertion_point(field_get:scan_context_io.RingKeys.data)
  return data_.Get(index);
}
inline ::scan_context_io::RingKey* RingKeys::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:scan_context_io.RingKeys.data)
  return data_.Mutable(index);
}
inline ::scan_context_io::RingKey* RingKeys::add_data() {
  // @@protoc_insertion_point(field_add:scan_context_io.RingKeys.data)
  return data_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::scan_context_io::RingKey >*
RingKeys::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:scan_context_io.RingKeys.data)
  return &data_;
}
inline const ::google::protobuf::RepeatedPtrField< ::scan_context_io::RingKey >&
RingKeys::data() const {
  // @@protoc_insertion_point(field_list:scan_context_io.RingKeys.data)
  return data_;
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace scan_context_io

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_ring_5fkeys_2eproto__INCLUDED