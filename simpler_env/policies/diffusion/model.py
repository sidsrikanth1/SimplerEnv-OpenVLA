from typing import Optional, Sequence, List
import os
import matplotlib.pyplot as plt
import numpy as np
from transforms3d.euler import euler2axangle
from collections import deque
from PIL import Image
import torch
import cv2 as cv
from simpler_env.utils.action.action_ensemble import ActionEnsembler
import numpy as np
import torch
from robomimic.algo import algo_factory
from easydict import EasyDict as edict

from robomimic.config.base_config import BaseConfig

import robomimic.utils.obs_utils as ObsUtils
import robomimic.utils.file_utils as FileUtils

class DiffusionInference:
    def __init__(
        self,
        saved_model_path: str = "pretrained/pi0",
        unnorm_key: Optional[str] = None,
        exec_horizon: int = 1,
        image_size: list[int] = [224, 224],
        action_scale: float = 1.0,
        action_ensemble_temp: float = -0.8,
        device: str = "cuda:0",
        **kwargs
    ) -> None:
        self.device = device
        os.environ["TOKENIZERS_PARALLELISM"] = "false"

        self.image_size = image_size
        self.action_scale = action_scale
        self.obs_horizon = 1
        self.obs_interval = 1
        self.pred_action_horizon = 5
        self.image_history = deque(maxlen=self.obs_horizon)
        self.exec_horizon = exec_horizon

        self.sticky_action_is_on = False
        self.gripper_action_repeat = 0
        self.sticky_gripper_action = 0.0
        self.previous_gripper_action = None

        self.action_ensemble = False
        self.action_ensemble_temp = action_ensemble_temp
        
        self.prev_obs = None

        if self.action_ensemble:
            self.action_ensembler = ActionEnsembler(
                self.pred_action_horizon, self.action_ensemble_temp
            )
        else:
            self.action_ensembler = None

        self.task = None
        self.task_description = None
        
        print("Loading model and config...")
        policy, ckpt_dict = FileUtils.policy_from_checkpoint(ckpt_path=saved_model_path, device=device, verbose=True)
        self.model = policy

    def _get_default_config(self):
        """Get default config for BC model"""
        config_path = "/workspace/robomimic/robomimic/exps/simpler_env.json"
        
        if os.path.exists(config_path):
            import json
            with open(config_path, 'r') as f:
                config = json.load(f)
            from easydict import EasyDict as edict
            return edict(config)
        else:
            # Minimal default config
            from easydict import EasyDict as edict
            return edict({
                "algo_name": "bc",
                "observation": {
                    "modalities": {
                        "obs": {
                            "low_dim": ["robot0_eef_pos", "robot0_eef_quat", "robot0_gripper_qpos"],
                            "rgb": ["agentview_image"]
                        }
                    }
                }
            })

    def reset(self, task_description: str) -> None:
        """Reset the model state for a new episode"""
        self.image_history.clear()
        if self.action_ensemble:
            self.action_ensembler.reset()
        
        self.model.start_episode()
        
        self.task_description = task_description
        self.sticky_action_is_on = False
        self.gripper_action_repeat = 0
        self.sticky_gripper_action = 0.0
        self.previous_gripper_action = None
        
        self.prev_obs = None
        
    def step(
        self, image: np.ndarray, task_description: Optional[str] = None, obs=None, *args, **kwargs
    ) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
        new_obs = {} # construct the observation dictionary as necessary
        new_obs["agentview_image"] = torch.from_numpy(image).float().to(self.device)
        new_obs["robot0_eef_pos"] = torch.from_numpy(obs["proprio"][:3]).float().to(self.device)
        new_obs["robot0_eef_quat"] = torch.from_numpy(obs["proprio"][3:7]).float().to(self.device)
        new_obs["robot0_gripper_qpos"] = torch.from_numpy(obs["proprio"][7:8]).float().to(self.device)

        one_hot_instruction = { # temporarily one-hot-encode the instructions
            "pick coke can": [1, 0, 0, 0, 0],
            "pick sponge": [0, 1, 0, 0, 0],
            "pick apple": [0, 0, 1, 0, 0],
            "open top drawer": [0, 0, 0, 1, 0],
            "close bottom drawer": [0, 0, 0, 0, 1],
        }[task_description]

        new_obs["task"] = torch.tensor(one_hot_instruction + [0,0,0], dtype=torch.float32).to(self.device)
        # new_new_obs = {
        #     "obs": {
        #         "low_dim": [
        #             new_obs["robot0_eef_pos"],
        #             new_obs["robot0_eef_quat"],
        #             new_obs["robot0_gripper_qpos"],
        #             new_obs["task"],
        #         ],
        #         "rgb": [
        #             new_obs["agentview_image"],
        #         ],
        #     }
        # }
        
        
        for k in new_obs.keys():
            if self.prev_obs is not None:
                new_obs[k] = torch.stack([self.prev_obs[k], new_obs[k]])
            else:
                new_obs[k] = torch.stack([new_obs[k], new_obs[k]]) # add the same obs twice for the first step
        
        for k,v in new_obs.items():
            print(k, v.shape)

        raw_action = self.model(ob=new_obs)
        
        # Convert to the expected format
        action = {
            "world_vector": raw_action[:3],
            "rot_axangle": raw_action[3:6], 
            "gripper": raw_action[6:7],
        }
        
        action["terminate_episode"] = np.array([0.0])
        return raw_action, action

    def preprocess_widowx_proprio(self, eef_pos):
        """Preprocess proprioceptive data for widowx setup"""
        # This is a placeholder - implement based on your specific preprocessing needs
        # For now, just return the eef_pos as is
        if isinstance(eef_pos, (list, tuple)):
            return np.array(eef_pos)
        elif isinstance(eef_pos, np.ndarray):
            return eef_pos
        else:
            return np.array([eef_pos])

    def _resize_image(self, image: np.ndarray) -> np.ndarray:
        image = cv.resize(image, tuple(self.image_size), interpolation=cv.INTER_AREA)
        return image

    def _add_image_to_history(self, image: np.ndarray) -> None:
        if len(self.image_history) == 0:
            self.image_history.extend([image] * self.obs_horizon)
        else:
            self.image_history.append(image)

    def _obtain_image_history(self) -> List[Image.Image]:
        image_history = list(self.image_history)
        images = image_history[:: self.obs_interval]
        # images = [Image.fromarray(image).convert("RGB") for image in images]
        return images

    def visualize_epoch(
        self,
        predicted_raw_actions: Sequence[np.ndarray],
        images: Sequence[np.ndarray],
        save_path: str,
    ) -> None:
        images = [self._resize_image(image) for image in images]
        ACTION_DIM_LABELS = ["x", "y", "z", "roll", "pitch", "yaw", "grasp"]

        img_strip = np.concatenate(np.array(images[::3]), axis=1)

        # set up plt figure
        figure_layout = [["image"] * len(ACTION_DIM_LABELS), ACTION_DIM_LABELS]
        plt.rcParams.update({"font.size": 12})
        fig, axs = plt.subplot_mosaic(figure_layout)
        fig.set_size_inches([45, 10])

        # plot actions
        pred_actions = np.array(
            [
                np.concatenate(
                    [a["world_vector"], a["rotation_delta"], a["open_gripper"]], axis=-1
                )
                for a in predicted_raw_actions
            ]
        )
        for action_dim, action_label in enumerate(ACTION_DIM_LABELS):
            # actions have batch, horizon, dim, in this example we just take the first action for simplicity
            axs[action_label].plot(
                pred_actions[:, action_dim], label="predicted action"
            )
            axs[action_label].set_title(action_label)
            axs[action_label].set_xlabel("Time in one episode")

        axs["image"].imshow(img_strip)
        axs["image"].set_xlabel("Time in one episode (subsampled)")
        plt.legend()
        plt.savefig(save_path)
