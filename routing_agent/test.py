from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict
from simple_interfaces.msg import StampedString

def main():
    # Dictionary with message data
    data = {"data": "{123:123}"}

    # Create a message instance
    msg = StampedString()

    # Populate message fields
    set_message_fields(msg, data)
    ans=message_to_ordereddict(msg)

    print(msg)  # Output: id: 123, name: "Robot"
    print(ans)
    print(ans["data"])
