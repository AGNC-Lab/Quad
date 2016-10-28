////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/serializable.h                                                              //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_SERIALIZABLE_H
#define UTILITIES_SERIALIZABLE_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class serializable {
    // List of pure virtual functions:
    // (1) uint32 size() const
    // (2) uint8* serialize(uint8* buff) const
    // (3) const uint8* deserialize(const uint8* buff)
    // (4) std::ostream& base_payload_to_stream(std::ostream& stream) const

    public:
      static const uint32 WIDTH = 3; // default format integer width
      static const uint32 PRECISION = 2; // default format float precision
      static const char DELIMITER = ','; // default format delimiter

    public:
      virtual ~serializable() {
        // Do nothing.
      }
      virtual uint32 size() const = 0;
      virtual uint8* serialize(uint8* buff) const = 0;
      virtual const uint8* deserialize(const uint8* buff) = 0;
    protected:
      virtual std::ostream& base_payload_to_stream(std::ostream& stream) const = 0;

    friend std::ostream& operator<<(std::ostream& stream, const serializable& obj) {
      return obj.base_payload_to_stream(stream);
    }
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
