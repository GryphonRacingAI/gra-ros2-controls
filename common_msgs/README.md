## common_msgs
Standardized message types for communication between modules.

### Types
Messages: 
- `msg/Cone.msg`
- `msg/ConeArray.msg`

### Message Specifications

#### Cone.msg
| Field | Type |
|-------|------|
| `position` | `geometry_msgs/Point` |
| `type` | `uint8` |

**Cone Type Constants:**
| Value | Constant |
|-------|----------|
| `0` | `UNKNOWN` |
| `1` | `YELLOW` |
| `2` | `BLUE` |
| `3` | `ORANGE` |
| `4` | `LARGE_ORANGE` |

#### ConeArray.msg
| Field | Type |
|-------|------|
| `header` | `std_msgs/Header` |
| `unknown_cones` | `common_msgs/Cone[]` |
| `yellow_cones` | `common_msgs/Cone[]` |
| `blue_cones` | `common_msgs/Cone[]` |
| `orange_cones` | `common_msgs/Cone[]` |
| `large_orange_cones` | `common_msgs/Cone[]` |

### Usage
- Python:
  ```python
  from common_msgs.msg import Cone
  from common_msgs.msg import ConeArray
  ```
- C++:
  ```cpp
  #include <common_msgs/msg/cone.hpp>
  #include <common_msgs/msg/cone_array.hpp>
  ```

- Add to dependencies (package.xml/CMake) as usual.

#### C++ Example
```cpp
#include <common_msgs/msg/cone.hpp>

using namespace common_msgs::msg::Cone;

// Create a cone message
Cone cone_msg;
cone_msg.position.x = 1.5;
cone_msg.position.y = 2.0;
cone_msg.type = YELLOW;

// Create cone array
ConeArray cone_array;
cone_array.yellow_cones.push_back(cone_msg);
```

#### Python Example
```python
from common_msgs.msg import Cone, ConeArray

# Create a cone message
cone_msg = Cone()
cone_msg.position.x = 1.5
cone_msg.position.y = 2.0
cone_msg.type = Cone.YELLOW

# Create cone array
cone_array = ConeArray()
cone_array.yellow_cones.append(cone_msg)
```