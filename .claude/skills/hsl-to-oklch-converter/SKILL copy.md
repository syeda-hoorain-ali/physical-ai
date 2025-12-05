---
name: hsl-to-oklch-converter
description: This skill converts HSL color values to OKLCH color values. It can process single or multiple colors, provided either directly by the user or from a specified file.
---

This skill automates the conversion of HSL color definitions to the OKLCH color space, leveraging Playwright to interact with the oklch.com web tool. It handles both individual color strings and batch conversions from files, replacing original HSL values with their OKLCH equivalents.

## When to Use This Skill
Use this skill when:
- You need to convert one or more HSL color values to OKLCH.
- You have a file containing HSL color definitions that need to be updated to OKLCH.
- You want to ensure color consistency and leverage the perceptually uniform properties of OKLCH.

## How to Use This Skill

To use this skill, follow these steps:

1.  **Launch Playwright and Navigate:**
    - Use the Playwright tool to navigate to `https://oklch.com`.
    - Example: `mcp__playwright__browser_navigate(url='https://oklch.com')`

2.  **Select HSL Input Mode:**
    - Click the dropdown to select the HSL input format.
    - Example: `mcp__playwright__browser_click(selector='div.dropdown-menu > button:has-text("HSL")')`

3.  **Perform Color Conversion:**
    - **For direct HSL input:**
        - Use the Playwright tool to type the HSL color value into the input field.
        - Example: `mcp__playwright__browser_fill_form(selector='input[type="text"]', value='hsl(180, 20%, 8%)')`
        - Use the Playwright tool to read the OKLCH output from the result field.
        - Example: `mcp__playwright__browser_evaluate(expression='document.querySelector("input[type='text'][placeholder*='oklch']").value')`
        - Return the OKLCH value to the user.

    - **For file input (e.g., D:/physical-ai/book-source/src/css/globals.css):**
        - Read the file content using the `Read` tool.
        - Example: `default_api.Read(file_path='D:/physical-ai/book-source/src/css/globals.css')`
        - Use the `Grep` tool with a regular expression to find all HSL color values in the file.
        - Example: `default_api.Grep(pattern='hsl\\(.*?\\)', path='D:/physical-ai/book-source/src/css/globals.css', output_mode='content')`
        - For each found HSL value:
            - Use the Playwright tool to type the HSL color into the input field (as described for direct input above).
            - Use the Playwright tool to read the OKLCH output (as described for direct input above).
            - Use the `Edit` tool to replace the original HSL value with the new OKLCH value in the file.
            - Example: `default_api.Edit(file_path='D:/physical-ai/book-source/src/css/globals.css', old_string='hsl(180, 20%, 8%)', new_string='oklch(0.162 0.006 196.56)', replace_all=True)`
        - After all conversions, inform the user that the file has been updated.

4.  **Repeat for Multiple Colors:** If there are more colors to convert, repeat step 3 for each color. If processing a file, the iteration will be handled within the file processing logic.
