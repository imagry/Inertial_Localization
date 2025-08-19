# AI Agent Instructions: Google Test Integration

This document contains the constant instructions for the AI agent assisting with writing code for this repository. The agent must follow these rules strictly during all interactions.
We are working on a localization algorithm for an autinomouse vehicle. This repo operates as a library used by production code. documentation  on the algorithm and code structure in README.md. 
## 1. Core Principles
* **One Item at a Time:** We will focus on a single, high-level task at a time.
* **Request for Instructions:** Before starting any new item, you MUST ask me for detailed instructions and goals for that specific item.
* **Confirm Before Action:** Before you perform ANY action (e.g., "I will now suggest adding `$FetchContent_Declare$` to `CMakeLists.txt`"), you MUST state your intended action and wait for my explicit confirmation ('yes', 'ok', 'proceed').
* **Single Documentation File:** Immediately after an action is completed, you MUST propose a summary of it for the **`implementation_notes.md`** file. You are not to create any other `.md` or documentation files.

## 2. Standard Workflow
Our process for every item will be:
1.  **Initiate:** I will state the high-level item I want to work on.
2.  **Inquire:** You will ask for detailed instructions.
3.  **Plan:** Based on my instructions, you will propose a specific action. 
4.  **Ask:** is my instructions are unclear, ask for clarification.
5.  **Confirm:** You will wait for my confirmation before proceeding with that action.
6.  **Execute:** You will execute the confirmed action.
7.  **Document:** You will provide the markdown entry for `implementation_notes.md` for my approval.
8.  **Loop:** You will propose the next logical action for the current item, and we repeat from step 4.

## 4. Documentation Format
All entries in `implementation_notes.md` should follow this format:
```markdown
### YYYY-MM-DD: [Component Name or File]
* **Action:** [Brief, clear description of the specific action taken.]
* **Details:** [Why the action was taken, what was changed. e.g., "Added Google Test dependency using FetchContent in the root CMakeLists.txt." or "Implemented unit test for `MyClass::calculate()` covering null input."]
* **Files Modified:** `CMakeLists.txt`, `tests/test_my_class.cpp`