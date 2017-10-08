package org.thingml.generated.network;

import org.thingml.generated.messages.*;
import org.thingml.java.ext.Event;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
public class SerialBinaryProtocol implements BinaryJava {
private static final PressureMessageType PRESSURE = new PressureMessageType((short) 20);
@Override
public Event instantiate(Byte[] payload) {
ByteBuffer buffer = ByteBuffer.wrap(JavaBinaryHelper.toPrimitive(payload));
buffer.order(ByteOrder.BIG_ENDIAN);
final short code = buffer.getShort();
switch(code) {
case 20:{
final int a = buffer.getInt();
final int b = buffer.getInt();
return PRESSURE.instantiate(a, b);
}
default: return null;
}
}
@Override
public Byte[] format(Event e){
switch(e.getType().getCode()){
default: return null;
}
}


}
