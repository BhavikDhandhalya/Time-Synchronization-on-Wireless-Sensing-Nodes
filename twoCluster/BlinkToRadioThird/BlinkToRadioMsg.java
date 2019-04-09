/**
 * This class is automatically generated by mig. DO NOT EDIT THIS FILE.
 * This class implements a Java interface to the 'BlinkToRadioMsg'
 * message type.
 */

public class BlinkToRadioMsg extends net.tinyos.message.Message {

    /** The default size of this message type in bytes. */
    public static final int DEFAULT_MESSAGE_SIZE = 24;

    /** The Active Message type associated with this message. */
    public static final int AM_TYPE = 6;

    /** Create a new BlinkToRadioMsg of size 24. */
    public BlinkToRadioMsg() {
        super(DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /** Create a new BlinkToRadioMsg of the given data_length. */
    public BlinkToRadioMsg(int data_length) {
        super(data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new BlinkToRadioMsg with the given data_length
     * and base offset.
     */
    public BlinkToRadioMsg(int data_length, int base_offset) {
        super(data_length, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new BlinkToRadioMsg using the given byte array
     * as backing store.
     */
    public BlinkToRadioMsg(byte[] data) {
        super(data);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new BlinkToRadioMsg using the given byte array
     * as backing store, with the given base offset.
     */
    public BlinkToRadioMsg(byte[] data, int base_offset) {
        super(data, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new BlinkToRadioMsg using the given byte array
     * as backing store, with the given base offset and data length.
     */
    public BlinkToRadioMsg(byte[] data, int base_offset, int data_length) {
        super(data, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new BlinkToRadioMsg embedded in the given message
     * at the given base offset.
     */
    public BlinkToRadioMsg(net.tinyos.message.Message msg, int base_offset) {
        super(msg, base_offset, DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new BlinkToRadioMsg embedded in the given message
     * at the given base offset and length.
     */
    public BlinkToRadioMsg(net.tinyos.message.Message msg, int base_offset, int data_length) {
        super(msg, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
    /* Return a String representation of this message. Includes the
     * message type name and the non-indexed field values.
     */
    public String toString() {
      String s = "Message <BlinkToRadioMsg> \n";
      try {
        s += "  [nodeid=0x"+Long.toHexString(get_nodeid())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "  [counter=0x"+Long.toHexString(get_counter())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "  [sendts=0x"+Long.toHexString(get_sendts())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "  [receivets=0x"+Long.toHexString(get_receivets())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "  [resendts=0x"+Long.toHexString(get_resendts())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "  [endts=0x"+Long.toHexString(get_endts())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      return s;
    }

    // Message-type-specific access methods appear below.

    /////////////////////////////////////////////////////////
    // Accessor methods for field: nodeid
    //   Field type: long, unsigned
    //   Offset (bits): 0
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'nodeid' is signed (false).
     */
    public static boolean isSigned_nodeid() {
        return false;
    }

    /**
     * Return whether the field 'nodeid' is an array (false).
     */
    public static boolean isArray_nodeid() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'nodeid'
     */
    public static int offset_nodeid() {
        return (0 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'nodeid'
     */
    public static int offsetBits_nodeid() {
        return 0;
    }

    /**
     * Return the value (as a long) of the field 'nodeid'
     */
    public long get_nodeid() {
        return (long)getUIntBEElement(offsetBits_nodeid(), 32);
    }

    /**
     * Set the value of the field 'nodeid'
     */
    public void set_nodeid(long value) {
        setUIntBEElement(offsetBits_nodeid(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'nodeid'
     */
    public static int size_nodeid() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'nodeid'
     */
    public static int sizeBits_nodeid() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: counter
    //   Field type: long, unsigned
    //   Offset (bits): 32
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'counter' is signed (false).
     */
    public static boolean isSigned_counter() {
        return false;
    }

    /**
     * Return whether the field 'counter' is an array (false).
     */
    public static boolean isArray_counter() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'counter'
     */
    public static int offset_counter() {
        return (32 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'counter'
     */
    public static int offsetBits_counter() {
        return 32;
    }

    /**
     * Return the value (as a long) of the field 'counter'
     */
    public long get_counter() {
        return (long)getUIntBEElement(offsetBits_counter(), 32);
    }

    /**
     * Set the value of the field 'counter'
     */
    public void set_counter(long value) {
        setUIntBEElement(offsetBits_counter(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'counter'
     */
    public static int size_counter() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'counter'
     */
    public static int sizeBits_counter() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: sendts
    //   Field type: long, unsigned
    //   Offset (bits): 64
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'sendts' is signed (false).
     */
    public static boolean isSigned_sendts() {
        return false;
    }

    /**
     * Return whether the field 'sendts' is an array (false).
     */
    public static boolean isArray_sendts() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'sendts'
     */
    public static int offset_sendts() {
        return (64 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'sendts'
     */
    public static int offsetBits_sendts() {
        return 64;
    }

    /**
     * Return the value (as a long) of the field 'sendts'
     */
    public long get_sendts() {
        return (long)getUIntBEElement(offsetBits_sendts(), 32);
    }

    /**
     * Set the value of the field 'sendts'
     */
    public void set_sendts(long value) {
        setUIntBEElement(offsetBits_sendts(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'sendts'
     */
    public static int size_sendts() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'sendts'
     */
    public static int sizeBits_sendts() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: receivets
    //   Field type: long, unsigned
    //   Offset (bits): 96
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'receivets' is signed (false).
     */
    public static boolean isSigned_receivets() {
        return false;
    }

    /**
     * Return whether the field 'receivets' is an array (false).
     */
    public static boolean isArray_receivets() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'receivets'
     */
    public static int offset_receivets() {
        return (96 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'receivets'
     */
    public static int offsetBits_receivets() {
        return 96;
    }

    /**
     * Return the value (as a long) of the field 'receivets'
     */
    public long get_receivets() {
        return (long)getUIntBEElement(offsetBits_receivets(), 32);
    }

    /**
     * Set the value of the field 'receivets'
     */
    public void set_receivets(long value) {
        setUIntBEElement(offsetBits_receivets(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'receivets'
     */
    public static int size_receivets() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'receivets'
     */
    public static int sizeBits_receivets() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: resendts
    //   Field type: long, unsigned
    //   Offset (bits): 128
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'resendts' is signed (false).
     */
    public static boolean isSigned_resendts() {
        return false;
    }

    /**
     * Return whether the field 'resendts' is an array (false).
     */
    public static boolean isArray_resendts() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'resendts'
     */
    public static int offset_resendts() {
        return (128 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'resendts'
     */
    public static int offsetBits_resendts() {
        return 128;
    }

    /**
     * Return the value (as a long) of the field 'resendts'
     */
    public long get_resendts() {
        return (long)getUIntBEElement(offsetBits_resendts(), 32);
    }

    /**
     * Set the value of the field 'resendts'
     */
    public void set_resendts(long value) {
        setUIntBEElement(offsetBits_resendts(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'resendts'
     */
    public static int size_resendts() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'resendts'
     */
    public static int sizeBits_resendts() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: endts
    //   Field type: long, unsigned
    //   Offset (bits): 160
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'endts' is signed (false).
     */
    public static boolean isSigned_endts() {
        return false;
    }

    /**
     * Return whether the field 'endts' is an array (false).
     */
    public static boolean isArray_endts() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'endts'
     */
    public static int offset_endts() {
        return (160 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'endts'
     */
    public static int offsetBits_endts() {
        return 160;
    }

    /**
     * Return the value (as a long) of the field 'endts'
     */
    public long get_endts() {
        return (long)getUIntBEElement(offsetBits_endts(), 32);
    }

    /**
     * Set the value of the field 'endts'
     */
    public void set_endts(long value) {
        setUIntBEElement(offsetBits_endts(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'endts'
     */
    public static int size_endts() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'endts'
     */
    public static int sizeBits_endts() {
        return 32;
    }

}
