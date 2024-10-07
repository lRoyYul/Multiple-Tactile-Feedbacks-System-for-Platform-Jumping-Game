using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

public class moveblock : MonoBehaviour
{
    public float moveRange = 5.0f; 
    public float speed = 2.0f;
    private bool isMoving;
    private bool isStanding;
    private Vector3 startPosition;
    private int direction;
    private Vector3 originalScale;
    private player player;
    private int k;

    public void ScaleY(float percentage)
    {
        // 计算新的Z轴尺寸并保持X和Y轴尺寸不变
        transform.localScale = new Vector3(originalScale.x, originalScale.y*(1-percentage/k), originalScale.z);
    }
    void Start()
    {
        
        startPosition = transform.position;
        originalScale = transform.localScale;
        isMoving = Random.Range(0, 2) == 1; 
        direction = Random.Range(0, 2); 
        if(this.gameObject.name.Contains("smooth"))
        {
            k = 1;
        }
        if (this.gameObject.name.Contains("rough"))
        {
            k = 15;
        }
        if (this.gameObject.name.Contains("block"))
        {
            k = 5;
        }
    }

    void Update()
    {
        if (isMoving)
        {
            float pingPong = Mathf.PingPong(Time.time * speed, moveRange);
            if (direction == 0)
            {
                transform.position = new Vector3(startPosition.x + pingPong, startPosition.y, startPosition.z);
            }
            else
            {
                transform.position = new Vector3(startPosition.x, startPosition.y, startPosition.z + pingPong);
            }
        }
        if(isStanding)
        {
            ScaleY(player.forcePercent);
        }
    }
    void OnCollisionEnter(Collision collision)
    {
        isMoving = false;
        isStanding = true;
        player = collision.gameObject.GetComponent<player>();
        
    }
}
