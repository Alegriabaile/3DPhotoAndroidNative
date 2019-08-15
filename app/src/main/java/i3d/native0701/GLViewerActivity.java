package i3d.native0701;

import android.content.Intent;
import android.graphics.Camera;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;

public class GLViewerActivity extends AppCompatActivity implements View.OnTouchListener {

    private MyGLViewer myGLViewer;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_glviewer);

        myGLViewer = findViewById(R.id.gl_viewer);
        myGLViewer.setOnTouchListener(this);


        ImageView up = (ImageView)findViewById(R.id.up);
        up.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View view) {
                GLUtils.translateCamera(GLUtils.UP, 1.0f);
                myGLViewer.requestRender();
            }
        });

        ImageView down = (ImageView)findViewById(R.id.down);
        down.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View view) {
                GLUtils.translateCamera(GLUtils.DOWN, 1.0f);
                myGLViewer.requestRender();
            }
        });

        ImageView left = (ImageView)findViewById(R.id.left);
        left.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View view) {
                GLUtils.translateCamera(GLUtils.LEFT, 1.0f);
                myGLViewer.requestRender();
            }
        });

        ImageView right = (ImageView)findViewById(R.id.right);
        right.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View view) {
                GLUtils.translateCamera(GLUtils.RIGHT, 1.0f);
                myGLViewer.requestRender();
            }
        });


        ImageView reset = (ImageView)findViewById(R.id.reset);
        reset.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
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
    public boolean onTouch(View view, MotionEvent event) {
        Log.d("////////////////////////GLViewerActivity::onTouch////////////////////////", "before myGLViewer.requestRender()");
        myGLViewer.requestRender();
        return true;
    }
}
