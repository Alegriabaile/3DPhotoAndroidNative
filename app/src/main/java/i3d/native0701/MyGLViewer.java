package i3d.native0701;

import android.content.Context;
import android.graphics.PixelFormat;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.util.Log;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

public class MyGLViewer extends GLSurfaceView {

    public MyGLViewer(Context context, AttributeSet attrs) {
        super(context, attrs);

        setEGLContextClientVersion(3);
        setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        getHolder().setFormat(PixelFormat.TRANSLUCENT);
        setRenderer(new Renderer() {
            @Override
            public void onSurfaceCreated(GL10 gl10, EGLConfig eglConfig) {
                GLUtils.initialize();
                if(GLUtils.isInitialized())
                    Log.e("/////////////////////MyGLViewer::onSurfaceCreated///////////////////////", " GLUtils isInitialized...");
                else
                    Log.d("/////////////////////MyGLViewer::onSurfaceCreated///////////////////////", " GLUtils is not Initialized...");
            }

            @Override
            public void onSurfaceChanged(GL10 gl10, int width, int height) {
                Log.d("/////////////////////MyGLViewer::onSurfaceChanged///////////////////////", "before reshape...");
                GLUtils.reshape(width, height);
            }

            @Override
            public void onDrawFrame(GL10 gl10) {
                Log.d("/////////////////////MyGLViewer::onDrawFrame///////////////////////", "before display...");
                GLUtils.display();
            }
        });
        setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
        setZOrderOnTop(true);
    }
}
