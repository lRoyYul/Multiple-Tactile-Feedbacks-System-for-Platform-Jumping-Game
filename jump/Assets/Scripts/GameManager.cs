using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class GameManager : MonoBehaviour
{
    public void StartGame()
    {
        SceneManager.LoadScene("MainScene"); 
    }
    void Start()
    {
        StartCoroutine(LoadNextSceneAfterDelay(1));  // �ȴ�1�������һ������
    }

    IEnumerator LoadNextSceneAfterDelay(float delay)
    {
        yield return new WaitForSeconds(delay);  // �ȴ�ָ������ʱ
        SceneManager.LoadScene("MainScene");
    }
}
