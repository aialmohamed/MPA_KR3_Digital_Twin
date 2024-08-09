import yaml
import re

class YamlToLatexConverter:
    def __init__(self, yaml_file, latex_file):
        self.yaml_file = yaml_file
        self.latex_file = latex_file

    def escape_latex_special_chars(self, text):
        """
        Escapes LaTeX special characters in the given text.
        """
        text = re.sub(r'`?<xref:([^>]+)>`?', r'\\textbf{\1}', text)
        special_chars = ['$', '_', '%', '&', '#', '~', '^']  
        for char in special_chars:
            text = text.replace(char,'\\' + char)
        return text

    def extract_name_description_datatype(self, data, parent_name=None):
        results = []
        
        for item in data:
            full_name = item.get('name', 'N/A')
            if parent_name:
                full_name = f"{parent_name}.{full_name}"
                
            name = self.escape_latex_special_chars(full_name)
            description = self.escape_latex_special_chars(item.get('description', 'N/A'))
            data_type = self.escape_latex_special_chars(item.get('data-type', 'N/A'))
            results.append((name, description, data_type))
            
            if 'fields' in item:
                results.extend(self.extract_name_description_datatype(item['fields'], parent_name=full_name))
        
        return results

    def convert(self):
        with open(self.yaml_file, 'r') as file:
            yaml_data = yaml.safe_load(file)
        
        variables = yaml_data.get('variables', [])
        name_description_datatype = self.extract_name_description_datatype(variables)

        with open(self.latex_file, 'w') as file:
            table_count = 0

            for i in range(0, len(name_description_datatype), 18):
                table_count += 1

                # Add \newpage before each new table
                if table_count > 1:  # Avoid \newpage before the first table
                    file.write('\\newpage\n')

                file.write(f'\\begin{{tabular}}{{|p{{0.3\\textwidth}}|p{{0.5\\textwidth}}|p{{0.2\\textwidth}}|}}\n')
                file.write('\\hline\n')
                file.write(f'\\textbf{{Table {table_count}: Name}} & \\textbf{{Description}} & \\textbf{{Datatype}} \\\\ \\hline\n')

                for name, description, data_type in name_description_datatype[i:i+18]:
                    if '.' in name:
                        indented_name = '\\quad ' + name.split('.', 1)[1]
                    else:
                        indented_name = name
                    file.write(f'{indented_name} & {description} & {data_type} \\\\ \\hline\n')

                file.write('\\end{tabular}\n\n')

if __name__ == "__main__":
    yaml_file = "Software/yaml_to_text/System.Variables.yml"
    output_file = "Software/yaml_to_text/table.tex"
    converter = YamlToLatexConverter(yaml_file, output_file)
    converter.convert()
