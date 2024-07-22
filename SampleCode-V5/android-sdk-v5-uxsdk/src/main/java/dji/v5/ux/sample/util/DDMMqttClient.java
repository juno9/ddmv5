package dji.v5.ux.sample.util;

import android.content.Context;
import android.os.Build;
import android.util.Log;


import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.MqttPersistenceException;
import org.json.JSONObject;

import java.io.UnsupportedEncodingException;
import java.net.URI;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class DDMMqttClient implements MqttCallback {

    private static final String TAG = "DDM_MQTT_CLIENT";


    public DDMMqttClient() {
    }

    public static DDMMqttClient getMqttClient(Context context
            , String brokerIp
            , String port
            , String clientId
            , String clientName
            , String clientPw
                                              ,String topic
    ) {
        return new DDMMqttClient().init(clientName, clientPw, brokerIp, port, clientId,topic);
    }

    public static DDMMqttClient getSimpleMqttClient(Context context
            , String brokerIp
            , String port
            , String topic
    ) throws Exception {
        // TOOD connect
        return new DDMMqttClient().connectMqtt(brokerIp, port, topic);
    }

    public DDMMqttClient(Consumer<HashMap<Object, Object>> fnc) {
        this.FNC = fnc;
        this.isRun = false;
    }

    /**
     * 사용자이름, 비밀번호, 주소, 접속후에 사용할 아이디값 입니다.
     */
    public DDMMqttClient init(String userName, String password, String serverURI, String port, String clientId,String topic) {
        option = new MqttConnectOptions();
        option.setCleanSession(true);
        option.setKeepAliveInterval(30);
        option.setUserName(userName);
        option.setPassword(password.toCharArray());
        String url = "tcp://" + serverURI + ":" + port;
        try {
            client = new MqttClient(url, clientId);
            client.setCallback(this);

            client.connect(option);
        client.subscribe(topic);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return this;
    }

    private DDMMqttClient connectMqtt(String brokerIp, String port, String topic) throws Exception{
        // TODO..
        String url = "tcp://"+brokerIp+":"+port;

        try {
            client = new MqttClient(url, MqttClient.generateClientId(), null);
            client.connect();
            client.subscribe(topic);
            client.setCallback(new MqttCallback() {
                @Override
                public void connectionLost(Throwable cause) {
                    Log.d(TAG,"Mqtt ReConnect");
//                try{connectMqtt();}catch(Exception e){Log.d(TAG,"MqttReConnect Error");}
                }
                @Override
                public void messageArrived(String topic, MqttMessage message) throws Exception {

                    // TODO Image Pub
//                JSONObject json = new JSONObject(new String(message.getPayload(), "UTF-8"));
//                chatAdapter.add(new ChatItem(json.getString("id"), json.getString("content")));
//                runOnUiThread(new Runnable() {
//                    @Override
//                    public void run() {
//                        chatAdapter.notifyDataSetChanged();
//                    }
//                });
                }
                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {

                }
            });
        } catch (Exception e) {
            e.printStackTrace();
        }

        return this;
    }


    public MqttConnectOptions getOption() {
        return option;
    }

    /***
     * 전송 메소드
     *
     **/
    public boolean sender(String topic, String msg) throws MqttPersistenceException, MqttException {
        MqttMessage message = new MqttMessage();
        message.setPayload(msg.getBytes());
//        message.setQos(1);
        client.publish(topic, message);
        Log.d(TAG, "Mqtt message published");
        return true;
    }

    public boolean senderJsonMap(String topic, Map map) throws MqttPersistenceException, MqttException, UnsupportedEncodingException {
        JSONObject json = new JSONObject(map);
        Log.d(TAG, "senderJsonMap :: json  " + json.toString());
        MqttMessage message = new MqttMessage(json.toString().getBytes("utf-8"));
        Log.d(TAG,"senderJsonMap :: MqttMessage  "+ message.getPayload());
        client.publish(topic, message);
        return true;
    }

    public boolean isRun() {
        return isRun;
    }

    public void setRun(boolean isRun) {
        this.isRun = isRun;
    }

    public MqttClient getClient() {
        return client;
    }

    public void setClient(MqttClient client) {
        this.client = client;
    }

    /**
     * 종료메소드입니다.<br>
     * 클라이언트를 종료 합니다.
     */
    public void close() {
        if (client != null) {
            try {
                client.disconnect();
                client.close();
            } catch (MqttException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void connectionLost(Throwable arg0) {
        if (FNC2 != null) {
            HashMap<Object, Object> result = new HashMap<>();
            result.put("result", arg0);
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                FNC2.accept(result);
            }
            arg0.printStackTrace();
        }
    }



    @Override
    public void deliveryComplete(IMqttDeliveryToken arg0) {
        if (FNC3 != null) {
            HashMap<Object, Object> result = new HashMap<>();
            try {
                result.put("result", arg0);
            } catch (Exception e) {
                e.printStackTrace();
                result.put("result", "ERROR");
                result.put("error", e.getMessage());
            }
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                FNC3.accept(result);
            }
        }
    }

    //메시지 도착
    @Override
    public void messageArrived(String arg0, MqttMessage arg1) throws Exception {
        if (FNC != null) {
            HashMap<Object, Object> result = new HashMap<>();
            result.put("topic", arg0);
            result.put("message", new String(arg1.getPayload(), "UTF-8"));
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                FNC.accept(result);  //콜백행위 실행
            }
        }
    }

    public void sendChat(String chat) {
        String id = "ExId";
        String content = chat;
        if (content.equals("")) {
        } else {
            JSONObject json = new JSONObject();
            try {
                json.put("id", id);
                json.put("content", content);
//                mqttClient.publish(TOPIC,new MqttMessage(json.toString().getBytes()));
            } catch (Exception e) {
            }
//            chatEditText.setText("");
        }
    }

    public Map<String, Object> getArgOptions() {
        return argOptions;
    }

    public void setArgOptions(Map<String, Object> argOptions) {
        this.argOptions = argOptions;
    }

    private String[] getAuth(URI uri) {
        String a = uri.getAuthority();
        String[] first = a.split("@");
        return first[0].split(":");
    }

    /***
     * 구독 대상 배열
     *
     * **/
    public boolean subscribe(String... topics) {
        try {
            if (topics != null) {
                for (String topic : topics) {
                    client.subscribe(topic, 0);
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
        return true;
    }

    /**
     * 커넥션이 끊어진 이후의 콜백행위를 등록합니다.<br>
     * 해쉬맵 형태의 결과에 키는 result, 값은 Throwable 객체를 반환 합니다.
     **/
    public void initConnectionLost(Consumer<HashMap<Object, Object>> fnc) {
        FNC2 = fnc;
    }

    /**
     * 커넥션이 끊어진 이후의 콜백행위를 등록합니다.<br>
     * 해쉬맵 형태의 결과에 키는 result, 값은 IMqttDeliveryToken 객체를 반환 합니다.
     **/
    public void initDeliveryComplete(Consumer<HashMap<Object, Object>> fnc) {
        FNC3 = fnc;
    }

    /**
     * Data Member..
     */
    private MqttClient client;
    private MqttConnectOptions option;
    private Map<String, Object> argOptions;
    private boolean isRun;
    private Consumer<HashMap<Object, Object>> FNC = null;  //메시지 도착 후 응답하는 함수
    private Consumer<HashMap<Object, Object>> FNC2 = null; //커넥션이 끊긴 후 응답하는 함수
    private Consumer<HashMap<Object, Object>> FNC3 = null; //전송이 완료된 이후 응답하는 함수.

}
