package dji.v5.ux.MAVLink.ardupilotmega;

import dji.v5.ux.MAVLink.MAVLinkPacket;
import dji.v5.ux.MAVLink.Messages.MAVLinkMessage;
import dji.v5.ux.MAVLink.Messages.MAVLinkPayload;

public class msg_power_pack_data extends MAVLinkMessage {

	
	public static final int MAVLINK_MSG_ID_POWER_PACK_DATA = 425;
    public static final int MAVLINK_MSG_ID_POWER_PACK_DATA_CRC = 209; //52;
    public static final int MAVLINK_MSG_LENGTH = 44;
    private static final long serialVersionUID = MAVLINK_MSG_ID_POWER_PACK_DATA;
    
    /**
    *  
    */
    public int count;
      
    /**
    * 1~65535.
    */
    public int product_number;
      
    /**
    * 0~350.0(bar).
    */
    public int h2;
      
    /**
    * 0~60.00V
    */
    public int output_voltage;
      
    /**
    * 0~100.00A
    */
    public int output_current;
      
    /**
    * 0~60.00V
    */
    public int battery_voltage;
      
    /**
    * -75.00~75.00A(+75)
    */
    public int battery_current;
      
    /**
    * -40.0~100.00(+20)
    */
    public int powerpack_temperature;
      
    /**
    * 0~80.0V
    */
    public int fuelcell1_voltage;
      
    /**
    * -40~100.0(+40)
    */
    public int fuelcell1_temperature1;
      
    /**
    * -40~100.0(+40)
    */
    public int fuelcell1_temperature2;
      
    /**
    * 0~50.00A
    */
    public int fuelcell1_current;
      
    /**
    * 0~100
    */
    public int fuelcell1_fan_speed;
      
    /**
    * 0~80.0V
    */
    public int fuelcell2_voltage;
      
    /**
    * -40~100.0(+40)
    */
    public int fuelcell2_temperature1;
      
    /**
    * -40~100.0(+40)
    */
    public int fuelcell2_temperature2;
      
    /**
    * 0~50.00A
    */
    public int fuelcell2_current;
      
    /**
    * 0~100
    */
    public int fuelcell2_fan_speed;
      
    /**
    * 254(fixed).
    */
    public short head;
      
    /**
    * 0~9.
    */
    public short powerpack_state;
      
    /**
    * TBD.
    */
    public short error_code;
      
    /**
    * 0~9.
    */
    public short fuelcell1_stack1_state;
      
    /**
    * 0~20.0A
    */
    public short fuelcell1_fan_current;
      
    /**
    * 0~9.
    */
    public short fuelcell2_stack2_state;
      
    /**
    * 0~20.0A
    */
    public short fuelcell2_fan_current;
      
    /**
    * 255(fixed)
    */
    public short tail;
    /**
    * Generates the payload for a mavlink message for a message of this type
    * @return
    */
    public MAVLinkPacket pack(){
        MAVLinkPacket packet = new MAVLinkPacket(MAVLINK_MSG_LENGTH);
        packet.sysid = 0;
        packet.compid = 0;
        packet.msgid = MAVLINK_MSG_ID_POWER_PACK_DATA;
        packet.crc_extra = MAVLINK_MSG_ID_POWER_PACK_DATA_CRC;
              
        packet.payload.putUnsignedShort(count);
              
        packet.payload.putUnsignedShort(product_number);
              
        packet.payload.putUnsignedShort(h2);
              
        packet.payload.putUnsignedShort(output_voltage);
              
        packet.payload.putUnsignedShort(output_current);
              
        packet.payload.putUnsignedShort(battery_voltage);
              
        packet.payload.putUnsignedShort(battery_current);
              
        packet.payload.putUnsignedShort(powerpack_temperature);
              
        packet.payload.putUnsignedShort(fuelcell1_voltage);
              
        packet.payload.putUnsignedShort(fuelcell1_temperature1);
              
        packet.payload.putUnsignedShort(fuelcell1_temperature2);
              
        packet.payload.putUnsignedShort(fuelcell1_current);
              
        packet.payload.putUnsignedShort(fuelcell1_fan_speed);
              
        packet.payload.putUnsignedShort(fuelcell2_voltage);
              
        packet.payload.putUnsignedShort(fuelcell2_temperature1);
              
        packet.payload.putUnsignedShort(fuelcell2_temperature2);
              
        packet.payload.putUnsignedShort(fuelcell2_current);
              
        packet.payload.putUnsignedShort(fuelcell2_fan_speed);
              
        packet.payload.putUnsignedByte(head);
              
        packet.payload.putUnsignedByte(powerpack_state);
              
        packet.payload.putUnsignedByte(error_code);
              
        packet.payload.putUnsignedByte(fuelcell1_stack1_state);
              
        packet.payload.putUnsignedByte(fuelcell1_fan_current);
              
        packet.payload.putUnsignedByte(fuelcell2_stack2_state);
              
        packet.payload.putUnsignedByte(fuelcell2_fan_current);
              
        packet.payload.putUnsignedByte(tail);
        
        return packet;
    }
    
    /**
     * Decode a powerpack_message message into this class fields
     *
     * @param payload The message to decode
     */
     public void unpack(MAVLinkPayload payload) {
         payload.resetIndex();
               
         this.count = payload.getUnsignedShort();
               
         this.product_number = payload.getUnsignedShort();
               
         this.h2 = payload.getUnsignedShort();
               
         this.output_voltage = payload.getUnsignedShort();
               
         this.output_current = payload.getUnsignedShort();
               
         this.battery_voltage = payload.getUnsignedShort();
               
         this.battery_current = payload.getUnsignedShort();
               
         this.powerpack_temperature = payload.getUnsignedShort();
               
         this.fuelcell1_voltage = payload.getUnsignedShort();
               
         this.fuelcell1_temperature1 = payload.getUnsignedShort();
               
         this.fuelcell1_temperature2 = payload.getUnsignedShort();
               
         this.fuelcell1_current = payload.getUnsignedShort();
               
         this.fuelcell1_fan_speed = payload.getUnsignedShort();
               
         this.fuelcell2_voltage = payload.getUnsignedShort();
               
         this.fuelcell2_temperature1 = payload.getUnsignedShort();
               
         this.fuelcell2_temperature2 = payload.getUnsignedShort();
               
         this.fuelcell2_current = payload.getUnsignedShort();
               
         this.fuelcell2_fan_speed = payload.getUnsignedShort();
               
         this.head = payload.getUnsignedByte();
               
         this.powerpack_state = payload.getUnsignedByte();
               
         this.error_code = payload.getUnsignedByte();
               
         this.fuelcell1_stack1_state = payload.getUnsignedByte();
               
         this.fuelcell1_fan_current = payload.getUnsignedByte();
               
         this.fuelcell2_stack2_state = payload.getUnsignedByte();
               
         this.fuelcell2_fan_current = payload.getUnsignedByte();
               
         this.tail = payload.getUnsignedByte();
         
     }

     /**
     * Constructor for a new message, just initializes the msgid
     */
     public msg_power_pack_data(){
         msgid = MAVLINK_MSG_ID_POWER_PACK_DATA;
     }

     /**
     * Constructor for a new message, initializes the message with the payload
     * from a mavlink packet
     *
     */
     public msg_power_pack_data(MAVLinkPacket mavLinkPacket){
         this.sysid = mavLinkPacket.sysid;
         this.compid = mavLinkPacket.compid;
         this.msgid = MAVLINK_MSG_ID_POWER_PACK_DATA;
         unpack(mavLinkPacket.payload);        
     }
     
     /**
      * Returns a string with the MSG name and data
      */
      public String toString(){
          return "MAVLINK_MSG_ID_POWERPACK_MESSAGE - sysid:"+sysid+" compid:"+compid+" count:"+count+" product_number:"+product_number+" h2:"+h2+" output_voltage:"+output_voltage+" output_current:"+output_current+" battery_voltage:"+battery_voltage+" battery_current:"+battery_current+" powerpack_temperature:"+powerpack_temperature+" fuelcell1_voltage:"+fuelcell1_voltage+" fuelcell1_temperature1:"+fuelcell1_temperature1+" fuelcell1_temperature2:"+fuelcell1_temperature2+" fuelcell1_current:"+fuelcell1_current+" fuelcell1_fan_speed:"+fuelcell1_fan_speed+" fuelcell2_voltage:"+fuelcell2_voltage+" fuelcell2_temperature1:"+fuelcell2_temperature1+" fuelcell2_temperature2:"+fuelcell2_temperature2+" fuelcell2_current:"+fuelcell2_current+" fuelcell2_fan_speed:"+fuelcell2_fan_speed+" head:"+head+" powerpack_state:"+powerpack_state+" error_code:"+error_code+" fuelcell1_stack1_state:"+fuelcell1_stack1_state+" fuelcell1_fan_current:"+fuelcell1_fan_current+" fuelcell2_stack2_state:"+fuelcell2_stack2_state+" fuelcell2_fan_current:"+fuelcell2_fan_current+" tail:"+tail+"";
      }
}
