package i3d.native0701;

import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;


public class MainActivity extends AppCompatActivity implements AdapterView.OnItemSelectedListener{

    static {
        System.loadLibrary("native-lib");
    }

    public static int COLS;
    public static int ROWS;

    private EditText editText;
    private ImageView imageView;
    private Bitmap bitmap;
    private TextView stateTextView, timerTextView;

    private Button mButtonGenerate3d, mButtonGLViewer;

    private String javaString = "/sdcard/Native0701/casual3d5";
    private Spinner mSpinner;
    private Context mContext;
    private List<String> mList;

    static private Handler handler;
    static private ProcessTask processTask;//防止MainActivity无法回收导致的资源泄露

    private Timer timer = new Timer();
    private int otime, lasttime, curtime, cnt;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mContext = MainActivity.this;

//        editText = findViewById(R.id.edittext_dir);
//        editText.setText(javaString);
        mSpinner = findViewById(R.id.my_spinner);
        mSpinner.setOnItemSelectedListener(this);
        mList = getFilesAllName("/sdcard/Native0701");
        ArrayAdapter<String> adapter = new ArrayAdapter(this, R.layout.my_spinner_item, R.id.text, mList);
        mSpinner.setAdapter(adapter);
        mSpinner.setPrompt("选择数据集...");

        imageView = findViewById(R.id.image_view);
        stateTextView = findViewById(R.id.textview_state);
        timerTextView = findViewById(R.id.textview_timer);

        handler = new Handler(new Handler.Callback() {
            @Override
            public boolean handleMessage(Message msg) {
                myToast(msg.what);
                return false;
            }
        });



        mButtonGenerate3d = findViewById(R.id.button_generate3d);
        mButtonGenerate3d.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v) {

//                javaString = editText.getText().toString();

                if(processTask!=null)
                    processTask.cancel(true);
                processTask = new ProcessTask(handler, javaString);

                processTask.execute();
                mButtonGenerate3d.setEnabled(false);
                mButtonGLViewer.setEnabled(false);
//                mButtonGenerate3d.setVisibility(View.GONE);

                TimerTask timerTask = new TimerTask() {
                    @Override
                    public void run() {
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                timerTextView.setText("总时间：" + getStringTime(cnt++));
                            }
                        });
                    }
                };
                cnt = 0;
                if (timer == null)
                    timer = new Timer();
                timer.schedule(timerTask,0,1000);
            }
        });


        mButtonGLViewer = findViewById(R.id.button_glviewer);
        mButtonGLViewer.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(MainActivity.this, GLViewerActivity.class);
                startActivity(intent);
            }
        });

    }

    @Override
    public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
        javaString = parent.getItemAtPosition(position).toString();
        Toast.makeText(mContext,"选择了数据集：" + javaString, Toast.LENGTH_SHORT).show();
    }

    @Override
    public void onNothingSelected(AdapterView<?> parent) {
    }

    //get all sub-dirs of "/sdcard/Native0701".
    public static List<String> getFilesAllName(String path) {

        List<String> s = new ArrayList<String>();

        File file=new File(path);
        File[] files=file.listFiles();

        for(int i =0;i<files.length;i++)
            s.add( files[i].getAbsolutePath());

        return s;
    }

    void myToast(int state)
    {
        switch (state){
            case 0:
                stateTextView.setText("started process...");
                break;
            case 1:
                stateTextView.setText("read and initialized data...");
                break;
            case 2:
                stateTextView.setText("estimated poses...");
                break;
            case 3:
                stateTextView.setText("warped to panoramas...");
                break;
            case 4:
                stateTextView.setText("stitched to one panorama...");
                break;
            case 5:
                stateTextView.setText("generated triangles...");
                break;
            case 10:
                ROWS = getImageH();
                COLS = getImageW();
                if(ROWS <= 0 || COLS <= 0)
                {
                    Log.e("MainActivity.OnCreate: ", "ROWS or COLS not positive...");
                    return;
                }

                bitmap = Bitmap.createBitmap(COLS, ROWS, Bitmap.Config.ARGB_8888);
                getImage(bitmap);
                imageView.setImageBitmap(bitmap);

                imageView.setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        bitmap = Bitmap.createBitmap(COLS, ROWS, Bitmap.Config.ARGB_8888);
                        getImage(bitmap);
                        imageView.setImageBitmap(bitmap);
                    }
                });
                stateTextView.setText("finished process...");
                mButtonGenerate3d.setEnabled(true);
                mButtonGLViewer.setEnabled(true);
                timer.cancel();
                timer = null;
                break;

            default:
                stateTextView.setText("error with code "+Integer.toString(state)+" ...");
                mButtonGenerate3d.setEnabled(true);
                mButtonGLViewer.setEnabled(true);
                timer.cancel();
                timer = null;
                break;
        }


    }
    private String getStringTime(int cnt) {
        int hour = cnt/3600;
        int min = cnt % 3600 / 60;
        int second = cnt % 60;
        return String.format(Locale.CHINA,"%02d:%02d:%02d",hour,min,second);
    }

//    public static native int processI3d(String javaString);
    public static native int getImageW();
    public static native int getImageH();
    public static native int getImage(Bitmap bitmap);
}
