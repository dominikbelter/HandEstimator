/** @file handest_defs.h
*
* Handest definitions
*
*/

#ifndef HANDEST_DEFS_H_INCLUDED
#define HANDEST_DEFS_H_INCLUDED

#include <cstdint>
#include <vector>

/// handest name space
namespace handest {

	/// handest default floating point
    typedef double float_type;

	/// 3 Element vector class
    class Vec3 {
    public:
        /// vector components
        union {
            struct {
                float_type v1, v2, v3;
            };
            struct {
                float_type x, y, z;
            };
            float_type v[3];
        };

        /// Default constructor sets the default configuration
        inline Vec3() {
            v1 = 0; v2=0; v3=0;
        }
    };

	/// Matrix representation of SO(3) group of rotations
    class Mat33 {
        public:
        /// Matrix elements
        union {
            struct {
                float_type m11, m12, m13;
                float_type m21, m22, m23;
                float_type m31, m32, m33;
            };
            float_type m[3][3];
        };
        /// Default constructor
        inline Mat33() {
            m11 = 0; m12 = 0; m13 = 0;
            m21 = 0; m22 = 0; m23 = 0;
            m31 = 0; m32 = 0; m33 = 0;
        }
    };

	/// Homogeneous representation of SE(3) rigid body transformations
    class Mat34 {
        public:
            /// rotation matrix
            Mat33 R;
            /// translation
            Vec3 p;

            /// Default constructor sets the default configuration
            inline Mat34() {
            }
    };

	/// RGBA colour space
    class RGBA {
        public:
            /// Colour representation
            union {
                struct {
                    std::uint8_t r, g, b, a;
                };
                std::uint8_t rgba_color[4];
            };
            /// Default constructor
            inline RGBA() {
                r = 255; g = 255; b = 255; a = 255;
            }
    };

	/// 3D point representation
    class Point3D {
        public:
            /// Point cloud
            typedef std::vector<Point3D> Cloud;

            /// Position
            Vec3 position;
            /// Colour
            RGBA colour;

            /// Default constructor
            inline Point3D() {
            }
    };

    /// Link representation
    class Link {
        public:
            /// Pose
            Mat34 pose;
            /// Point cloud
            Point3D::Cloud surface;
			/// Length
			float_type length;

            /// Default constructor
            inline Link() {
            }
    };

	/// Finger
    class Finger {
        public:
            /// Number of joints
            static const std::uint8_t JOINTS = 4;
            /// Number of links
            static const std::uint8_t LINKS = 3;

            /// Finger configuration
            class Config {
                public:
                    /// Config elements
                    float_type conf[JOINTS];

                    /// Default constructor
                    inline Config() {
                        std::fill(conf, conf + sizeof(conf), float_type(0.0));
                    }
            };

            /// Finger configuration + pose
            class Pose {
                public:

                    /// 3D pose of the base
                    Mat34 pose;
                    /// Configuration
                    Config config;
                    /// Shape representation
                    Link chain[LINKS];

                    /// Default constructor
                    inline Pose() {
                    }
            };
    };

	/// Hand
    class Hand {
        public:
            /// Number of fingers
            static const std::uint8_t FINGERS = 5;
            /// Number of joints per finger
            static const std::uintptr_t FINGER_JOINTS = Finger::JOINTS;
            /// Number of joints
            static const std::uintptr_t JOINTS = FINGERS*FINGER_JOINTS;

            /// Hand configuration
            class Config {
                public:

                /// Config elements
                union {
                    struct {
                        float_type thumb[FINGER_JOINTS];
                        float_type index[FINGER_JOINTS];
                        float_type middle[FINGER_JOINTS];
                        float_type ring[FINGER_JOINTS];
                        float_type pinky[FINGER_JOINTS];
                    };
                    float_type conf[JOINTS];
                };
                /// Default constructor sets the default configuration
                inline Config() {
                    std::fill(conf, conf + sizeof(conf), float_type(0.));
                }
            };

            /// Hand configuration + pose
            class Pose {
                public:

                    /// Configuration
                    Config config;
                    /// 3D pose of the base
                    Mat34 pose;
                    /// Palm
                    Link palm;
                    /// Fingers
                    Finger::Pose fingers[FINGERS];

                    /// Default constructor
                    inline Pose() {
                    }
            };
    };
}

#endif // HANDEST_DEFS_H_INCLUDED
