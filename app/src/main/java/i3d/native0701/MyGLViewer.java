package i3d.native0701;

import android.content.Context;
import android.graphics.PixelFormat;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

public class MyGLViewer extends GLSurfaceView implements View.OnTouchListener {

    private float oldX, oldY;
    private double oldDistance;
    private int hits;

    private double distance(MotionEvent event) {
        double dx = event.getX(0) - event.getX(1),
                dy = event.getY(0) - event.getY(1);
        return Math.sqrt(dx*dx + dy*dy);
    }

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

    @Override
    public boolean onTouch(View view, MotionEvent event) {
        int action = event.getAction() & MotionEvent.ACTION_MASK;
        if(MotionEvent.ACTION_DOWN == action) {
            oldX = event.getX();
            oldY = event.getY();
            hits = 1;
        } else if(MotionEvent.ACTION_UP == action) {
            hits = 0;
        } else if(MotionEvent.ACTION_POINTER_DOWN == action) {
            hits += 1;
            oldDistance = distance(event);
        } else if(MotionEvent.ACTION_POINTER_UP == action) {
            hits -= 1;
        } else if(MotionEvent.ACTION_MOVE == action) {
            if(hits >= 2) {
                double newDistance = distance(event);
                if(Math.abs(newDistance - oldDistance) > 5.0) {
                    GLUtils.zoomCamera((float)(newDistance-oldDistance)/10.0f);
                    requestRender();
                    oldDistance = newDistance;
                }
            } else {
                float dx = event.getX() - oldX, dy = event.getY() - oldY;
                double moveDistance = Math.sqrt(dx*dx + dy*dy);
                if(moveDistance > 5.0) {
                    GLUtils.rotateCamera(dx, -dy);
                    requestRender();
                    oldX += dx;
                    oldY += dy;
                }
            }
        } else {
            GLUtils.resetR();
            requestRender();
        }
        return true;
    }

}
