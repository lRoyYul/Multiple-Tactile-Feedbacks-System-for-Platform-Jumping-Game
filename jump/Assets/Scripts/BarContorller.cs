using UnityEngine;
using UnityEngine.UI; // 引用 UI 命名空间

public class BarController : MonoBehaviour
{
    public Slider progressBar; // 引用 UI 的 Slider
    public player playerScript;
    public Image backgroundImage;
    public void SetBackgroundTransparency(float alpha)
    {
        Color newColor = backgroundImage.color;
        newColor.a = alpha;
        backgroundImage.color = newColor;
    }
    void Start()
    {
        SetBackgroundTransparency(0.5f);  // 设置为50%透明
    }
    void Update()
    {
        if (playerScript != null && progressBar != null)
        {
            progressBar.value = playerScript.forcePercent;
        }
    }
}
