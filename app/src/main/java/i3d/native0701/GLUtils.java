package i3d.native0701;

public class GLUtils {
    static {
        System.loadLibrary("native-lib");
    }

    static int UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3;
    static int ROTX = 0, ROTY = 1, MOVENEAR = 2, MOVEFAR = 3, MOVEUP = 4, MOVEDOWN=5, MOVELEFT = 6, MOVERIGHT = 7;

    public static native void initialize();
    public static native boolean isInitialized();
    public static native void reshape(int width, int height);
    public static native void display();

    public static native void translateCamera(int camera_Movement, float step);
    public static native void rotateCamera(float xoffset, float yoffset);

    public static native void resetPose();

}