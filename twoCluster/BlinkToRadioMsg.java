/**
 * This class is automatically generated by mig. DO NOT EDIT THIS FILE.
 * This class implements a Java interface to the 'BlinkToRadioMsg'
 * message type.
 */

public class BlinkToRadioMsg extends net.tinyos.message.Message {

    /** The default size of this message type in bytes. */
    public static final int DEFAULT_MESSAGE_SIZE = 21;

    /** The Active Message type associated with this message. */
    public static final int AM_TYPE = 6;

    /** Create a new BlinkToRadioMsg of size 21. */
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
    // public String toString() {
    //   String s = "Message <BlinkToRadioMsg> \n";
    //   try {
    //     s += "  [nodeid=0x"+Long.toHexString(get_nodeid())+"]\n";
    //   } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
    //   try {
    //     s += "  [counter=0x"+Long.toHexString(get_counter())+"]\n";
    //   } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
    //   try {
    //     s += "  [sendTS=0x"+Long.toHexString(get_sendTS())+"]\n";
    //   } catch (ArrayIndexOutOfBoundsException aioobe) {  Skip field  }
    //   try {
    //     s += "  [receiveTS=0x"+Long.toHexString(get_receiveTS())+"]\n";
    //   } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
    //   try {
    //     s += "  [sendTS_of_member=0x"+Long.toHexString(get_sendTS_of_member())+"]\n";
    //   } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
    //   try {
    //     s += "  [receiveTS_of_head=0x"+Long.toHexString(get_receiveTS_of_head())+"]\n";
    //   } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
    //   return s;
    // }

    public String toString() {
      String s = "";
      try {
        s += get_nodeid()+"\t";
        //s += "  [nodeid=0x"+Long.toHexString(get_nodeid())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += get_counter()+"\t";
        //s += "  [counter=0x"+Long.toHexString(get_counter())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "sendTS = "+get_sendTS()+"\t";
        //s += "  [p_sendts=0x"+Long.toHexString(get_p_sendts())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "recTS = "+get_receiveTS()+"\t";
        //s += "  [receivets=0x"+Long.toHexString(get_receivets())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "sendTS_of_member = "+get_sendTS_of_member()+"\t";
        //s += "  [virtual_clk="+Float.toString(get_virtual_clk())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "receiveTS_of_head = "+get_receiveTS_of_head()+"\t";
        //s += "  [skew_cmp="+Float.toString(get_skew_cmp())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      return s;
    }

    // Message-type-specific access methods appear below.

    /////////////////////////////////////////////////////////
    // Accessor methods for field: nodeid
    //   Field type: short, unsigned
    //   Offset (bits): 0
    //   Size (bits): 8
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
     * Return the value (as a short) of the field 'nodeid'
     */
    public short get_nodeid() {
        return (short)getUIntBEElement(offsetBits_nodeid(), 8);
    }

    /**
     * Set the value of the field 'nodeid'
     */
    public void set_nodeid(short value) {
        setUIntBEElement(offsetBits_nodeid(), 8, value);
    }

    /**
     * Return the size, in bytes, of the field 'nodeid'
     */
    public static int size_nodeid() {
        return (8 / 8);
    }

    /**
     * Return the size, in bits, of the field 'nodeid'
     */
    public static int sizeBits_nodeid() {
        return 8;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: counter
    //   Field type: long, unsigned
    //   Offset (bits): 8
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
        return (8 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'counter'
     */
    public static int offsetBits_counter() {
        return 8;
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
    // Accessor methods for field: sendTS
    //   Field type: long, unsigned
    //   Offset (bits): 40
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'sendTS' is signed (false).
     */
    public static boolean isSigned_sendTS() {
        return false;
    }

    /**
     * Return whether the field 'sendTS' is an array (false).
     */
    public static boolean isArray_sendTS() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'sendTS'
     */
    public static int offset_sendTS() {
        return (40 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'sendTS'
     */
    public static int offsetBits_sendTS() {
        return 40;
    }

    /**
     * Return the value (as a long) of the field 'sendTS'
     */
    public long get_sendTS() {
        return (long)getUIntBEElement(offsetBits_sendTS(), 32);
    }

    /**
     * Set the value of the field 'sendTS'
     */
    public void set_sendTS(long value) {
        setUIntBEElement(offsetBits_sendTS(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'sendTS'
     */
    public static int size_sendTS() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'sendTS'
     */
    public static int sizeBits_sendTS() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: receiveTS
    //   Field type: long, unsigned
    //   Offset (bits): 72
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'receiveTS' is signed (false).
     */
    public static boolean isSigned_receiveTS() {
        return false;
    }

    /**
     * Return whether the field 'receiveTS' is an array (false).
     */
    public static boolean isArray_receiveTS() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'receiveTS'
     */
    public static int offset_receiveTS() {
        return (72 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'receiveTS'
     */
    public static int offsetBits_receiveTS() {
        return 72;
    }

    /**
     * Return the value (as a long) of the field 'receiveTS'
     */
    public long get_receiveTS() {
        return (long)getUIntBEElement(offsetBits_receiveTS(), 32);
    }

    /**
     * Set the value of the field 'receiveTS'
     */
    public void set_receiveTS(long value) {
        setUIntBEElement(offsetBits_receiveTS(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'receiveTS'
     */
    public static int size_receiveTS() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'receiveTS'
     */
    public static int sizeBits_receiveTS() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: sendTS_of_member
    //   Field type: long, unsigned
    //   Offset (bits): 104
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'sendTS_of_member' is signed (false).
     */
    public static boolean isSigned_sendTS_of_member() {
        return false;
    }

    /**
     * Return whether the field 'sendTS_of_member' is an array (false).
     */
    public static boolean isArray_sendTS_of_member() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'sendTS_of_member'
     */
    public static int offset_sendTS_of_member() {
        return (104 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'sendTS_of_member'
     */
    public static int offsetBits_sendTS_of_member() {
        return 104;
    }

    /**
     * Return the value (as a long) of the field 'sendTS_of_member'
     */
    public long get_sendTS_of_member() {
        return (long)getUIntBEElement(offsetBits_sendTS_of_member(), 32);
    }

    /**
     * Set the value of the field 'sendTS_of_member'
     */
    public void set_sendTS_of_member(long value) {
        setUIntBEElement(offsetBits_sendTS_of_member(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'sendTS_of_member'
     */
    public static int size_sendTS_of_member() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'sendTS_of_member'
     */
    public static int sizeBits_sendTS_of_member() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: receiveTS_of_head
    //   Field type: long, unsigned
    //   Offset (bits): 136
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'receiveTS_of_head' is signed (false).
     */
    public static boolean isSigned_receiveTS_of_head() {
        return false;
    }

    /**
     * Return whether the field 'receiveTS_of_head' is an array (false).
     */
    public static boolean isArray_receiveTS_of_head() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'receiveTS_of_head'
     */
    public static int offset_receiveTS_of_head() {
        return (136 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'receiveTS_of_head'
     */
    public static int offsetBits_receiveTS_of_head() {
        return 136;
    }

    /**
     * Return the value (as a long) of the field 'receiveTS_of_head'
     */
    public long get_receiveTS_of_head() {
        return (long)getUIntBEElement(offsetBits_receiveTS_of_head(), 32);
    }

    /**
     * Set the value of the field 'receiveTS_of_head'
     */
    public void set_receiveTS_of_head(long value) {
        setUIntBEElement(offsetBits_receiveTS_of_head(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'receiveTS_of_head'
     */
    public static int size_receiveTS_of_head() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'receiveTS_of_head'
     */
    public static int sizeBits_receiveTS_of_head() {
        return 32;
    }

}
