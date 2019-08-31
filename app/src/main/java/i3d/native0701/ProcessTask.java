package i3d.native0701;

import android.os.AsyncTask;
import android.os.Handler;

public class ProcessTask extends AsyncTask<String, Integer, String> {

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    private int process;
    private Handler handler;
    private String string;
    public native int processI3d(String _string);

    int notifyMainActivity()
    {
        handler.sendEmptyMessage(++process);
        return 0;
    }


    public ProcessTask(Handler _handler, String _string)
    {
        this.handler = _handler;
        this.string = _string;
    }

    @Override
    protected void onPreExecute() {
//        textView = findViewById(R.id.textView);
//        textView.setText("Multi thread 加载中");
        handler.sendEmptyMessage(0);
//        progressBar = (ProgressBar) findViewById(R.id.progress_bar);
//            text.setText("加载中");
        // 执行前显示提示
    }

    // 方法2：doInBackground（）
    // 作用：接收输入参数、执行任务中的耗时操作、返回 线程任务执行的结果
    // 此处通过计算从而模拟“加载进度”的情况
    @Override
    protected String doInBackground(String... params) {
        process = 0;
        processI3d(string);
        return null;
    }

    // 方法3：onProgressUpdate（）
    // 作用：在主线程 显示线程任务执行的进度
    @Override
    protected void onProgressUpdate(Integer... progresses) {

    }

    // 方法4：onPostExecute（）
    // 作用：接收线程任务执行结果、将执行结果显示到UI组件
    @Override
    protected void onPostExecute(String result) {
        // 执行完毕后，则更新UI
//        textView.setText("Multi thread 加载完毕");
        handler.sendEmptyMessage(10);
    }

    // 方法5：onCancelled()
    // 作用：将异步任务设置为：取消状态
    @Override
    protected void onCancelled() {

//        textView.setText("Multi thread 已取消");
//        progressBar.setProgress(0);
        handler.sendEmptyMessage(-1);
    }

}
