using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
//using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using UnityEngine.SocialPlatforms.Impl;
using TMPro;
using System.IO.Ports;
using System.Text;
using System;

using System.Linq;
//using System.Math;

public class player : MonoBehaviour
{
    public Vector3 offset;
    private Transform playerTransform;
    private Transform massCenter;
    public GameObject currentCube;
    private Rigidbody playerRigidbody;
    public Vector3 cubePlace;
    private float direction;
    public int maxCubeNumber;
    private Queue<GameObject> cubes = new Queue<GameObject>();
    bool isSpacePressed;// = false;
    private bool jump = false;
    private bool inTheAir = false;
    private float start = 0;
    public float speed;
    private int Sum=0;
    private float cubeDirection;
    public TextMeshProUGUI scoreText;
    private int cubeLength;
    public int score = -1;
    public TMP_InputField directionInputField;
    public float directionangle;
    public float instanttime;
    public GameObject[] cubePrefabs;
    public GameObject cubePrefab;
   // public GameObject directionObject;
    //public SerialReader directionInput;
    private SerialPort sp,jumpsp;
    public string Pdata;
    public string jumpdata;
    public float forcePercent;
    public string data;
    public string jdata;
    private int prefabIndex;
    private float forceback;
    public int previous_cube;
    private List<float> force_list = new List<float>();
    private List<float> estimate_list = new List<float>();
    private bool firstNonzero_element = false;
    private float press_force;
    private moveblock cubepercent;
    public float estimatetime;
    private float max_force;
    public float total_speed;
    private bool GAMEOVER;
    void Start()
    {
        sp = new SerialPort("COM5", 115200);
        jumpsp = new SerialPort("COM4", 115200);
        sp.Open();
        sp.ReadTimeout = 100;
        jumpsp.Open();
        jumpsp.ReadTimeout = 100;
       // directionInput = directionObject.GetComponent<SerialReader>();
        playerRigidbody = GetComponent<Rigidbody>();
        playerTransform = GetComponent<Transform>();
        massCenter = playerTransform.Find("masscenter");
        var cube = Instantiate(cubePrefab, cubePlace, Quaternion.identity);
        //currentCube = cube;
        cubes.Enqueue(currentCube);
        cubes.Enqueue(cube);
        playerRigidbody.centerOfMass = massCenter.localPosition;
        StartCoroutine(ReadSerialData());
        StartCoroutine(jumpReadSerialData());
    }
    IEnumerator ReadSerialData()
    {
        while (true)
        {
            if (sp.IsOpen && sp.BytesToRead > 0)
            {
                byte[] buffer = new byte[sp.BytesToRead];
                sp.Read(buffer, 0, buffer.Length);

                Pdata += System.Text.Encoding.UTF8.GetString(buffer);

                if (Pdata.Contains("\n"))
                {
                    string[] lines = Pdata.Split('\n');
                    foreach (string line in lines)
                    {

                        if (!string.IsNullOrEmpty(line) && line.Length>=7)
                        {
                            data = line;
                            //float floatdata=float.Parse(line);
                            //if (floatdata<90) { }
                            //Debug.Log("Unity Received data: " + data);
                            //count = 0;
                        }
                    }
                    Pdata = ""; // 清空已处理的数据
                }
            }
            if (sp.IsOpen)
            {
                sp.WriteLine(previous_cube.ToString()); // 向串行端口写数据

                sp.WriteLine(jdata.ToString());

                //sp.WriteLine(press_force.ToString());
                //Debug.Log("Data Sent: " + previous_cube);
            }

            yield return null;
        }
    }
    IEnumerator jumpReadSerialData()
    {
        while (true)
        {
            if (jumpsp.IsOpen && jumpsp.BytesToRead > 0)
            {
                byte[] buffer = new byte[jumpsp.BytesToRead];
                jumpsp.Read(buffer, 0, buffer.Length);

                jumpdata += System.Text.Encoding.UTF8.GetString(buffer);

                if (jumpdata.Contains("\n"))
                {
                    string[] lines = jumpdata.Split('\n');
                    foreach (string line in lines)
                    {

                        if (!string.IsNullOrEmpty(line) && line.Length >= 4)
                        {
                            jdata = line;
                            //float floatdata=float.Parse(line);
                            //if (floatdata<90) { }
                            Debug.Log("Unity Received data: " + line);
                            //Debug.Log("jump receive--------- " + line);
                            //count = 0;
                        }
                    }
                    jumpdata = ""; // 清空已处理的数据
                }
            }
            if (jumpsp.IsOpen)
            {
                jumpsp.WriteLine(previous_cube.ToString()); // 向串行端口写数据
                if (GAMEOVER == true) { 
                jumpsp.WriteLine(1.ToString());
                }

                GAMEOVER = false;

                //sp.WriteLine(press_force.ToString());
                //Debug.Log("Data Sent: " + previous_cube);
            }

            yield return null;
        }
    }
    async void Update()
    {
        try
        { direction = -float.Parse(data);
        } catch (Exception ex) { }
        
        start = Time.time;

        int current_kwall;
        if (previous_cube == 2)
        {
            current_kwall = 100;
        }
        else if (previous_cube == 1)
        {
            current_kwall = 500;
        }
        else
        {
            current_kwall = 1000;
        }
        //Debug.Log("---------"+jdata);
        press_force = -float.Parse(jdata);
        float xh = press_force / current_kwall;
        
        if ( press_force < 0 )
        {
            press_force = 0;
        }
        bool into_flag = false;
        force_list.Add(press_force);
        //Debug.Log("force:" + xh);

        directionangle = direction * (float)(System.Math.PI / 180) * 10;
        //Debug.Log("dir:" + directionangle + "-----" + direction);
        
        
        //Debug.Log("force:" + press_force);
        if (press_force > max_force)
        {
            max_force = press_force;
        }
        else
        {
            into_flag = true;
        }
        if (!firstNonzero_element && press_force != 0)
        {
            firstNonzero_element = true;
        }
        estimatetime = 2 * (max_force * speed) / (float)9.81;
        total_speed = max_force * speed;
        forcePercent = xh / (float)0.0235;
        if (press_force == 0 && firstNonzero_element)
        {
            //Debug.Log("force:" + force_list.Max());
            //Debug.Log("dir:" + directionangle);

            playerRigidbody.AddForce(new Vector3((float)(System.Math.Sin(directionangle)), 1,
            (float)(System.Math.Cos(directionangle))) * force_list.Max() * speed,
            ForceMode.Impulse);
            
            force_list.Clear(); 
            firstNonzero_element = false;
            max_force = 0;
            //Debug.Log("force:" + force_list.LastOrDefault(n => n != 0.0f));
            //Debug.Log("JUMPED");
        }
        /*
        if (isSpacePressed == false && Input.GetKey(KeyCode.Space)&& inTheAir==false)
        {
            Debug.Log("pressed!");
            jump=true;
            start = Time.time;
            isSpacePressed = true;
        }
        if(isSpacePressed==true)
        {
            instanttime = (Time.time - start) / 5;
            Debug.Log("time now" + instanttime);
        }

        if (Input.GetKeyUp(KeyCode.Space) && inTheAir == false)
        {
            instanttime = 0;
            float time = Time.time - start;
            Debug.Log("leave!   "+time);
            Debug.Log("jump Direction: " + direction);
            playerRigidbody.AddForce(new Vector3((float)(System.Math.Sin(directionangle)), 1, (float)(System.Math.Cos(directionangle))) * (time * speed), ForceMode.Impulse);
            inTheAir = true;
            isSpacePressed = false;
        }
        */
        scoreText.text = score.ToString();
    }

    private void NewCube()
    {
        var random = new System.Random();
        cubeDirection = UnityEngine.Random.Range(0,90);
        //Debug.Log("dire" + cubeDirection);
        cubeDirection = cubeDirection * (float)(System.Math.PI / 180);
        cubeLength = UnityEngine.Random.Range(5, 10);
        prefabIndex = random.Next(cubePrefabs.Length);
        cubePrefab = cubePrefabs[prefabIndex];
        
        var cube = Instantiate(cubePrefab, new Vector3( (float)(System.Math.Sin(cubeDirection))* cubeLength, 0 ,(float)(System.Math.Cos(cubeDirection))*cubeLength) + cubePlace, Quaternion.identity);
        Sum++;
        cubePlace = cube.transform.position;
        cubes.Enqueue(cube);
        if (cubes.Count > maxCubeNumber) Destroy(cubes.Dequeue());
    }

    private void OnCollisionEnter(Collision other)
    {
        if (other.gameObject.CompareTag("Ground") || other.gameObject == currentCube && jump)
        {
            // FindObjectOfType<GameManager>().EndGame();
            GAMEOVER = true;
            SceneManager.LoadScene("EndScene");
        }
        else if (other.gameObject != currentCube)
        {
            score++;
            inTheAir = false;

            NewCube();
            currentCube = other.gameObject;
            string otherObjectName = other.gameObject.name;
            cubepercent = currentCube.GetComponent<moveblock>();
            Debug.Log("Collided with: " + otherObjectName);
            if (otherObjectName.Contains("smooth"))
            {
                previous_cube = 2;
            }
            else if(otherObjectName.Contains("block") )
            {
                previous_cube = 0;
            }
            else
            {
                previous_cube = 1;
            }
            
        }
    }
    void OnDestroy()
    {
        if (sp != null && sp.IsOpen)
        {
            sp.Close();
        }
        if (jumpsp != null && jumpsp.IsOpen)
        {
            jumpsp.Close();
        }
    }
}
