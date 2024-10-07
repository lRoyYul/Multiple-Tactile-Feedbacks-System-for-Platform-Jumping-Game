using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class follow_player : MonoBehaviour
{

    public Transform player;
    public Vector3 offset;

    private void FixedUpdate()
    {
        transform.position = player.position + offset;
    }

}
