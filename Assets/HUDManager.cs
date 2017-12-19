using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HUDManager : MonoBehaviour {
	public static HUDManager instance;

	public GameObject SuccessText;
	public GameObject InvalidText;

	void Awake() {
		instance = this;
	}

	void Start() {
		Reset();
	}

	public void Reset() {
		SuccessText.SetActive(false);
		InvalidText.SetActive(false);
	}

	public void Success() {
		SuccessText.SetActive(true);
		InvalidText.SetActive(false);
	}

	public void Invalid() {
		SuccessText.SetActive(false);
		InvalidText.SetActive(true);
	}
}
