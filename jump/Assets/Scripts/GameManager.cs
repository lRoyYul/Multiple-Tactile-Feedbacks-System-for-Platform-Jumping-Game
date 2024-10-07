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
        StartCoroutine(LoadNextSceneAfterDelay(1));  // 等待1秒加载下一个场景
    }

    IEnumerator LoadNextSceneAfterDelay(float delay)
    {
        yield return new WaitForSeconds(delay);  // 等待指定的延时
        SceneManager.LoadScene("MainScene");
    }
}
