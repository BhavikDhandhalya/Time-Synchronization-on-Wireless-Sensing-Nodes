/**
 * This class is automatically generated by mig. DO NOT EDIT THIS FILE.
 * This class implements a Java interface to the 'BlinkToRadioMsg'
 * message type.
 */

public class BlinkToRadioMsg extends net.tinyos.message.Message {

    /** The default size of this message type in bytes. */
    public static final int DEFAULT_MESSAGE_SIZE = 25;

    /** The Active Message type associated with this message. */
    public static final int AM_TYPE = 6;

    /** Create a new BlinkToRadioMsg of size 25. */
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
        s += "sendTS = "+get_p_sendts()+"\t";
        //s += "  [p_sendts=0x"+Long.toHexString(get_p_sendts())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "recTS = "+get_receivets()+"\t";
        //s += "  [receivets=0x"+Long.toHexString(get_receivets())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "vir_clk = "+Float.toString(get_virtual_clk())+"\t";
        //s += "  [virtual_clk="+Float.toString(get_virtual_clk())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "skew = "+Float.toString(get_skew_cmp())+"\t";
        //s += "  [skew_cmp="+Float.toString(get_skew_cmp())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "off = "+Float.toString(get_offset_cmp())+"";
        //s += "  [offset_cmp="+Float.toString(get_offset_cmp())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      return s;
    }

    // Message-type-specific access methods appear below.

    /////////////////////////////////////////////////////////
    // Accessor methods for field: nodeid
    //   Field type: short
    //   Offset (bits): 0
    //   Size (bits): 8
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'nodeid' is signed (true).
     */
    public static boolean isSigned_nodeid() {
        return true;
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
    //   Field type: long
    //   Offset (bits): 8
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'counter' is signed (true).
     */
    public static boolean isSigned_counter() {
        return true;
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
    // Accessor methods for field: p_sendts
    //   Field type: long
    //   Offset (bits): 40
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'p_sendts' is signed (true).
     */
    public static boolean isSigned_p_sendts() {
        return true;
    }

    /**
     * Return whether the field 'p_sendts' is an array (false).
     */
    public static boolean isArray_p_sendts() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'p_sendts'
     */
    public static int offset_p_sendts() {
        return (40 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'p_sendts'
     */
    public static int offsetBits_p_sendts() {
        return 40;
    }

    /**
     * Return the value (as a long) of the field 'p_sendts'
     */
    public long get_p_sendts() {
        return (long)getUIntBEElement(offsetBits_p_sendts(), 32);
    }

    /**
     * Set the value of the field 'p_sendts'
     */
    public void set_p_sendts(long value) {
        setUIntBEElement(offsetBits_p_sendts(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'p_sendts'
     */
    public static int size_p_sendts() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'p_sendts'
     */
    public static int sizeBits_p_sendts() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: receivets
    //   Field type: long
    //   Offset (bits): 72
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'receivets' is signed (true).
     */
    public static boolean isSigned_receivets() {
        return true;
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
        return (72 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'receivets'
     */
    public static int offsetBits_receivets() {
        return 72;
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
    // Accessor methods for field: virtual_clk
    //   Field type: float
    //   Offset (bits): 104
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'virtual_clk' is signed (true).
     */
    public static boolean isSigned_virtual_clk() {
        return true;
    }

    /**
     * Return whether the field 'virtual_clk' is an array (false).
     */
    public static boolean isArray_virtual_clk() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'virtual_clk'
     */
    public static int offset_virtual_clk() {
        return (104 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'virtual_clk'
     */
    public static int offsetBits_virtual_clk() {
        return 104;
    }

    /**
     * Return the value (as a float) of the field 'virtual_clk'
     */
    public float get_virtual_clk() {
        return (float)getFloatElement(offsetBits_virtual_clk(), 32);
    }

    /**
     * Set the value of the field 'virtual_clk'
     */
    public void set_virtual_clk(float value) {
        setFloatElement(offsetBits_virtual_clk(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'virtual_clk'
     */
    public static int size_virtual_clk() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'virtual_clk'
     */
    public static int sizeBits_virtual_clk() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: skew_cmp
    //   Field type: float
    //   Offset (bits): 136
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'skew_cmp' is signed (true).
     */
    public static boolean isSigned_skew_cmp() {
        return true;
    }

    /**
     * Return whether the field 'skew_cmp' is an array (false).
     */
    public static boolean isArray_skew_cmp() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'skew_cmp'
     */
    public static int offset_skew_cmp() {
        return (136 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'skew_cmp'
     */
    public static int offsetBits_skew_cmp() {
        return 136;
    }

    /**
     * Return the value (as a float) of the field 'skew_cmp'
     */
    public float get_skew_cmp() {
        return (float)getFloatElement(offsetBits_skew_cmp(), 32);
    }

    /**
     * Set the value of the field 'skew_cmp'
     */
    public void set_skew_cmp(float value) {
        setFloatElement(offsetBits_skew_cmp(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'skew_cmp'
     */
    public static int size_skew_cmp() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'skew_cmp'
     */
    public static int sizeBits_skew_cmp() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: offset_cmp
    //   Field type: float
    //   Offset (bits): 168
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'offset_cmp' is signed (true).
     */
    public static boolean isSigned_offset_cmp() {
        return true;
    }

    /**
     * Return whether the field 'offset_cmp' is an array (false).
     */
    public static boolean isArray_offset_cmp() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'offset_cmp'
     */
    public static int offset_offset_cmp() {
        return (168 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'offset_cmp'
     */
    public static int offsetBits_offset_cmp() {
        return 168;
    }

    /**
     * Return the value (as a float) of the field 'offset_cmp'
     */
    public float get_offset_cmp() {
        return (float)getFloatElement(offsetBits_offset_cmp(), 32);
    }

    /**
     * Set the value of the field 'offset_cmp'
     */
    public void set_offset_cmp(float value) {
        setFloatElement(offsetBits_offset_cmp(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'offset_cmp'
     */
    public static int size_offset_cmp() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'offset_cmp'
     */
    public static int sizeBits_offset_cmp() {
        return 32;
    }

}
