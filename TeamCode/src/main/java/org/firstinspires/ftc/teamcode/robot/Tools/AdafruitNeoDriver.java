package org.firstinspires.ftc.teamcode.robot.Tools;

import android.graphics.Color;
import android.util.Log;
import androidx.annotation.ColorInt;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

// This code adapted from https://github.com/w8wjb/ftc-neodriver

@I2cDeviceType
@DeviceProperties(name = "Adafruit NeoDriver", xmlTag = "AdafruitNeoDriver", description = "an Adafruit NeoDriver board", builtIn = false)
public class AdafruitNeoDriver extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, AdafruitNeoDriver.Parameters>
        implements I2cAddrConfig, OpModeManagerNotifier.Notifications {

    public static final I2cAddr I2CADDR_DEFAULT = I2cAddr.create7bit(0x60);

    private static final byte   BASE_REGISTER_ADDR = 0x0E;

    /**
     * Number of the N
     */
    private static final byte NEOPIXEL_PIN = 15;

    /**
     * Maximum number of bytes that can be sent in one I2C frame
     */
    private static final int MAX_TX_BYTES = 24;  // LK: Why not 32? (Actually 30; first two bytes are address; note 3 bytes per LED)

    private static final String TAG = "NeoDriver";

    private enum FunctionRegister {

        STATUS(0x00),
        PIN(0x01),
        SPEED(0x02),
        BUF_LENGTH(0x03),
        BUF(0x04),
        SHOW(0x05);

        public final byte bVal;

        FunctionRegister(int i) {
            this.bVal = (byte) i;
        }
    }

    // LK added this constructor
    public AdafruitNeoDriver(I2cDeviceSynch deviceClient) {
        this(deviceClient, true, new Parameters());
    }

    public AdafruitNeoDriver(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        this(deviceClient, deviceClientIsOwned, new Parameters());
    }

    protected AdafruitNeoDriver(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned, @NonNull Parameters defaultParameters) {
        super(i2cDeviceSynch, deviceClientIsOwned, defaultParameters);

        this.deviceClient.setI2cAddress(I2CADDR_DEFAULT);
        this.deviceClient.setLogging(true);
        this.deviceClient.setLoggingTag("NeoDriverI2C");

        this.deviceClient.engage();
    }


    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "NeoDriver";
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        // no-op
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        // no-op
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        // no-op
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        // Turn all the lights off when the OpMode is stopped
        fill(0);
        show();
    }

    @Override
    protected boolean internalInitialize(@NonNull Parameters parameters) {

        RobotLog.vv(TAG, "internalInitialize()...");


        // Make sure we're talking to the correct I2c address
        this.deviceClient.setI2cAddress(parameters.i2cAddr);

        // Can't do anything if we're not really talking to the hardware
        if (!this.deviceClient.isArmed()) {
            Log.d(TAG, "not armed");
            return false;
        }

        byte[] bytes = new byte[]{FunctionRegister.PIN.bVal, NEOPIXEL_PIN};
        this.deviceClient.write(BASE_REGISTER_ADDR, bytes, I2cWaitControl.WRITTEN);

        RobotLog.vv(TAG, "Wrote NEOPIXEL_PIN");

        return true;
    }

    @Override
    public I2cAddr getI2cAddress() {
        return this.parameters.i2cAddr;
    }

    @Override
    public void setI2cAddress(I2cAddr newAddress) {
        this.parameters.i2cAddr = newAddress;
        this.deviceClient.setI2cAddress(newAddress);
    }

    /**
     * Sets the number of pixels that are available in the strand
     * @param numPixels
     */
    //@Override
    public void setNumberOfPixels(int numPixels) {

        parameters.numPixels = numPixels;

        int bufferLength = parameters.numPixels * parameters.bytesPerPixel;

        ByteBuffer buffer = ByteBuffer.allocate(3).order(ByteOrder.LITTLE_ENDIAN);
        buffer.put(FunctionRegister.BUF_LENGTH.bVal);
        buffer.putShort((short) bufferLength);

        byte[] bytes = buffer.array();
        Log.v("NeoDriver", "BUF_LENGTH " + toHex(bytes));
        deviceClient.write(BASE_REGISTER_ADDR, bytes, I2cWaitControl.WRITTEN);
    }

    /**
     * Sets a specific pixel in a strand to a color
     * @param index Index of pixel, 0-based
     * @param colorString Hex color string, i.e. #RRGGBB
     */
    //@Override
    public void setPixelColor(int index, String colorString) {
        setPixelColor(index, Color.parseColor(colorString));
    }

    /**
     * Sets a specific pixel in a strand to a color
     * @param index Index of pixel, 0-based
     * @param color Color encoded as an int. See {@link  android.graphics.Color} for useful color utilities
     */
    //@Override
    public void setPixelColor(int index, int color) {

        if (index > parameters.numPixels) {
            throw new ArrayIndexOutOfBoundsException("Index " + index + " is out of bounds of the pixel array");
        }

        int bufferSize = 3 + parameters.bytesPerPixel;

        ByteBuffer buffer = ByteBuffer.allocate(bufferSize).order(ByteOrder.BIG_ENDIAN);
        buffer.put(FunctionRegister.BUF.bVal);
        buffer.putShort((short) (index * parameters.bytesPerPixel));
        buffer.put(colorsToBytes(parameters.bytesPerPixel, parameters.colorOrder, color));

        byte[] bytes = buffer.array();
        deviceClient.write(BASE_REGISTER_ADDR, bytes);
    }

    /**
     * Sets a sequence of pixels to colors, starting at index 0
     * @param colorStrings 1 or more hex color strings, i.e. #RRGGBB
     */
    //@Override
    public void setPixelColors(String... colorStrings) {
        int[] colors = new int[colorStrings.length];
        for (int i = 0; i < colorStrings.length; i++) {
            colors[i] = Color.parseColor(colorStrings[i]);
        }
        setPixelColors(colors);
    }

    /**
     * Sets a sequence of pixels to colors, starting at index 0
     * @param colors Colors encoded as an int array. See {@link  android.graphics.Color} for useful color utilities
     */
    //@Override
    public void setPixelColors(@ColorInt int[] colors) {

        if (colors.length > parameters.numPixels) {
            throw new ArrayIndexOutOfBoundsException("Incoming color array is larger than the pixel array");
        }

        byte[] colorData = colorsToBytes(parameters.bytesPerPixel, parameters.colorOrder, colors);

        for (int chunkStart = 0; chunkStart < colorData.length; chunkStart += MAX_TX_BYTES) {
            int chunkLength = Math.min(MAX_TX_BYTES, colorData.length - chunkStart);
            sendPixelData((short) chunkStart, colorData, chunkStart, chunkLength);
        }
    }

    private void sendPixelData(short memOffset, byte[] colorData, int offset, int length) {
        int bufferSize = 3 + length;

        ByteBuffer buffer = ByteBuffer.allocate(bufferSize).order(ByteOrder.BIG_ENDIAN);
        buffer.put(FunctionRegister.BUF.bVal);
        buffer.putShort(memOffset);
        buffer.put(colorData, offset, length);

        byte[] bytes = buffer.array();
        deviceClient.write(BASE_REGISTER_ADDR, bytes);
    }

    //@Override
    public void fill(int color) {
        int[] colors = new int[parameters.numPixels];
        Arrays.fill(colors, color);
        setPixelColors(colors);
    }

    //@Override
    public void fill(String colorString) {
        fill(Color.parseColor(colorString));
    }

    /**
     * This must be called to signal that the color data should be sent to the NeoPixel strand
     */
    //@Override
    public void show() {
        byte[] bytes = new byte[]{FunctionRegister.SHOW.bVal};
        deviceClient.write(BASE_REGISTER_ADDR, bytes, I2cWaitControl.WRITTEN);
    }

    private static byte[] colorsToBytes(int bytesPerPixel, ColorOrder order, int... colors) {

        byte[] colorData = new byte[colors.length * bytesPerPixel];

        for (int colorIndex = 0; colorIndex < colors.length; colorIndex++) {
            int color = colors[colorIndex];
            int dataIndex = colorIndex * bytesPerPixel;

            colorData[dataIndex + order.redIndex] = (byte) (color >> 16);
            colorData[dataIndex + order.greenIndex] = (byte) (color >> 8);
            colorData[dataIndex + order.blueIndex] = (byte) color;
            if (bytesPerPixel == 4) {
                colorData[dataIndex + 3] = (byte) (color >> 24);
            }
        }

        return colorData;
    }

    private String toHex(byte[] bytes) {
        StringBuilder builder = new StringBuilder(bytes.length * 3);

        for (byte b : bytes) {
            builder.append(String.format(" %02X", b));
        }

        return builder.toString();
    }


    enum ColorOrder {
        RGB(0, 1, 2),
        RBG(0, 2, 1),
        GRB(1, 0, 2),
        GBR(2, 0, 1),
        BRG(1, 2, 0),
        BGR(2, 1, 0);

        private final int redIndex;
        private final int greenIndex;
        private final int blueIndex;

        private ColorOrder(int redIndex, int greenIndex, int blueIndex) {
            this.redIndex = redIndex;
            this.greenIndex = greenIndex;
            this.blueIndex = blueIndex;
        }
    }

    static class Parameters implements Cloneable {
        /**
         * the address at which the sensor resides on the I2C bus.
         */
        public I2cAddr i2cAddr = I2CADDR_DEFAULT;

        /**
         * Number of pixels in the string
         */
        public int numPixels = 1;

        /**
         * Number of bytes per pixel. Use 3 for RGB or 4 for RGBW
         */
        public int bytesPerPixel = 3;

        /**
         * Order that the pixel colors are in
         */
        public ColorOrder colorOrder = ColorOrder.GRB;


        @Override
        public Parameters clone() {
            try {
                return (Parameters) super.clone();
            } catch (CloneNotSupportedException e) {
                throw new AssertionError();
            }
        }
    }
}

/* LK Future work:

Maintain a list of the colors of each pixel at the NeoDriver (actual display)

//Maintain a list of changes requested (to Prioritize)?
//    -What if the list gets too long and never catches up?
//
//or Maintain a list of colors to be sent (pixel order not priority)?
//    -What if the changes keep happening early in the array so only that side gets updated regularly?
//    -Better idea might be to step through the list a little further every loop

Maintain a list of the colors of each pixel that we want to be displayed (local, future display)

Current concept:
0. Start at index 0

1. Look for the first LED needing a change
    a. Send over that LED plus the next x pixels (for a total of MAX_TX_BYTES bytes)
    b. Update the local list of the displayed LEDs
    b. Advance the counter by x pixels - exit to next loop

2. If reached the end, Execute show
    Assuming 320 pixels, and 10 per update, this would be 32 sends.  At 20ms per loop, that's a 640ms update.
    Another option would be to "show" periodically, every so many writes, for a higher refresh but with tearing

In this scenario, only one I2C transaction happens per loop.
However, if the robot is idle (no user input or sensitive state machines), you could update more than one transaction.

*/