package i3d.native0701;

import android.content.Intent;
import android.graphics.Camera;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.LinearLayout;

public class GLViewerActivity extends AppCompatActivity  implements View.OnTouchListener {

    private MyGLViewer myGLViewer;

    private float oldX, oldY;
    private double oldDistance;
    private int hits;


    private double distance(MotionEvent event) {
        double dx = event.getX(0) - event.getX(1),
                dy = event.getY(0) - event.getY(1);
        return Math.sqrt(dx*dx + dy*dy);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        try{
            this.getSupportActionBar().hide();
        }catch (NullPointerException e){}
        setContentView(R.layout.activity_glviewer);

        FrameLayout frameLayout = findViewById(R.id.frame_layout);
        frameLayout.getLayoutParams().height = frameLayout.getLayoutParams().width;


        myGLViewer = findViewById(R.id.gl_viewer);
        myGLViewer.setOnTouchListener(myGLViewer);


        LinearLayout linearLayout = findViewById(R.id.translate_camera);
        linearLayout.setOnTouchListener(this);

        LinearLayout reset = findViewById(R.id.linearlayout_reset);
        reset.setOnClickListener(new View.OnClickListener(){

            @Override
            public void onClick(View v) {
                GLUtils.resetPose();
                myGLViewer.requestRender();
            }
        });
    }




    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    }

    @Override
    protected void onResume() {
        super.onResume();
        myGLViewer.onResume();
    }

    @Override
    protected void onPause() {
        super.onPause();
        myGLViewer.onPause();
    }


    @Override
    public boolean onTouch(View v, MotionEvent event) {
        //translate camera
        int action = event.getAction() & MotionEvent.ACTION_MASK;
        if(MotionEvent.ACTION_DOWN == action) {
            oldX = event.getX();
            oldY = event.getY();
            hits = 1;
        } else if(MotionEvent.ACTION_UP == action) {
            hits = 0;
            GLUtils.resetT();
        } else if(MotionEvent.ACTION_MOVE == action) {
            float dx = event.getX() - oldX, dy = event.getY() - oldY;
            double moveDistance = Math.sqrt(dx*dx + dy*dy);
                GLUtils.translateCamera(GLUtils.UP, dy*50.0f);
                GLUtils.translateCamera(GLUtils.LEFT, dx*50.0f);
                myGLViewer.requestRender();
                oldX += dx;
                oldY += dy;
        }

        return true;
    }
}
