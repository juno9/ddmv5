package dji.v5.ux.sample.util.video;

public class DDMFrameInfo {

    private byte[] rawFrame;
    private int colorFormat;
    private int capacity;
    private int width;
    private int height;
    private int[] pixels;

    private int flags;
    private long duration;
    private long offset;
    private long offsetEnd;
    private long decodeTimestamp;
    private long presentationTimestamp;


    public int getWidth() {
        return width;
    }
    public void setWidth(int width) {
        this.width = width;
    }
    public int getHeight() {
        return height;
    }
    public void setHeight(int height) {
        this.height = height;
    }
    public int getColorFormat() {
        return colorFormat;
    }
    public void setColorFormat(int colorFormat) {
        this.colorFormat = colorFormat;
    }
    public byte[] getRawFrame() {
        return rawFrame;
    }
    public void setRawFrame(byte[] rawFrame) {
        this.rawFrame = rawFrame;
    }
    public int getCapacity() {
        return capacity;
    }
    public void setCapacity(int capacity) {
        this.capacity = capacity;
    }
    public int[] getPixels() {
        return pixels;
    }
    public void setPixels(int[] pixels) {
        this.pixels = pixels;
    }
    public int getFlags() {
        return flags;
    }
    public void setFlags(int flags) {
        this.flags = flags;
    }
    public long getDuration() {
        return duration;
    }
    public void setDuration(long duration) {
        this.duration = duration;
    }
    public long getOffset() {
        return offset;
    }
    public void setOffset(long offset) {
        this.offset = offset;
    }
    public long getOffsetEnd() {
        return offsetEnd;
    }
    public void setOffsetEnd(long offsetEnd) {
        this.offsetEnd = offsetEnd;
    }
    public long getDecodeTimestamp() {
        return decodeTimestamp;
    }
    public void setDecodeTimestamp(long decodeTimestamp) {
        this.decodeTimestamp = decodeTimestamp;
    }
    public long getPresentationTimestamp() {
        return presentationTimestamp;
    }
    public void setPresentationTimestamp(long presentationTimestamp) {
        this.presentationTimestamp = presentationTimestamp;
    }
}