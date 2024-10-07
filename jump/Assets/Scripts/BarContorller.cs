using UnityEngine;
using UnityEngine.UI; // ���� UI �����ռ�

public class BarController : MonoBehaviour
{
    public Slider progressBar; // ���� UI �� Slider
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
        SetBackgroundTransparency(0.5f);  // ����Ϊ50%͸��
    }
    void Update()
    {
        if (playerScript != null && progressBar != null)
        {
            progressBar.value = playerScript.forcePercent;
        }
    }
}
