#include "AST/CompoundStatement.hpp"
#include "AST/for.hpp"
#include "AST/function.hpp"
#include "AST/program.hpp"
#include "codegen/CodeGenerator.hpp"
#include "sema/SemanticAnalyzer.hpp"
#include "sema/SymbolTable.hpp"
#include "visitor/AstNodeInclude.hpp"

#include <algorithm>
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <iostream>
#include <cstring>
using namespace std;
CodeGenerator::CodeGenerator(
    const std::string &source_file_name,
    const std::string &save_path,
    std::unordered_map<SemanticAnalyzer::AstNodeAddr, SymbolManager::Table> &&p_symbol_table_of_scoping_nodes)
    : m_symbol_manager(false /* no dump */),
      m_source_file_path(source_file_name),
      m_symbol_table_of_scoping_nodes(std::move(p_symbol_table_of_scoping_nodes)) {

    const auto &real_path = save_path.empty() ? std::string{"."} : save_path;
    auto slash_pos = source_file_name.rfind('/');
    auto dot_pos = source_file_name.rfind('.');

    if (slash_pos != std::string::npos) {
        ++slash_pos;
    } else {
        slash_pos = 0;
    }
    auto output_file_path{
        real_path + "/" +
        source_file_name.substr(slash_pos, dot_pos - slash_pos) + ".S"};

    m_output_file.reset(fopen(output_file_path.c_str(), "w"));
    assert(m_output_file.get() && "Failed to open output file");
}

static void dumpInstructions(FILE *p_out_file, const char *format, ...) {
    va_list args;
    va_start(args, format);
    vfprintf(p_out_file, format, args);
    va_end(args);
}

void CodeGenerator::visit(ProgramNode &p_program) {
    constexpr const char *const riscv_assembly_file_prologue =
        "    .file \"%s\"\n"
        "    .option nopic\n";
    dumpInstructions(m_output_file.get(), riscv_assembly_file_prologue,
                     m_source_file_path.c_str());

     m_symbol_manager.pushScope(
        std::move(m_symbol_table_of_scoping_nodes.at(&p_program)));

    auto visit_ast_node = [&](auto &ast_node) { ast_node->accept(*this); };
    for_each(p_program.getDeclNodes().begin(), p_program.getDeclNodes().end(), visit_ast_node);
    for_each(p_program.getFuncNodes().begin(), p_program.getFuncNodes().end(), visit_ast_node);

    dumpInstructions(m_output_file.get(),
        ".section    .text\n"
        "    .align 2\n"
        "    .globl main\n"
        "    .type main, @function\n"
        "main:\n"
        "    addi sp, sp, -128\n"
        "    sw ra, 124(sp)\n"
        "    sw s0, 120(sp)\n"
        "    addi s0, sp, 128\n"
    );

    const_cast<CompoundStatementNode &>(p_program.getBody()).accept(*this);

    dumpInstructions(m_output_file.get(),
        "    lw ra, 124(sp)\n"
        "    lw s0, 120(sp)\n"
        "    addi sp, sp, 128\n"
        "    jr ra \n"
        "    .size main, .-main\n"
 
    );

    m_symbol_manager.popScope();
}


void CodeGenerator::visit(DeclNode &p_decl) {
    for (const auto &variable : p_decl.getVariables()) {
        const SymbolEntry *entry = m_symbol_manager.lookup(variable->getName());
        if (!entry) {
            assert(false && "Variable not found in symbol table");
            continue;
        }

        if (entry->getLevel() == 0 &&
            entry->getKind() == SymbolEntry::KindEnum::kConstantKind) {
            const Constant *constant = entry->getAttribute().constant();
            dumpInstructions(m_output_file.get(),
                ".section    .rodata\n"
                "    .align 2\n"
                "    .globl %s\n"
                "    .type %s, @object\n"
                "%s:\n"
                "    .word %s\n", // 直接放只讀段
                variable->getNameCString(),
                variable->getNameCString(),
                variable->getNameCString(),
                constant->getConstantValueCString()
            );

        } else if (entry->getLevel() == 0 &&
                   entry->getKind() == SymbolEntry::KindEnum::kVariableKind) {
            dumpInstructions(m_output_file.get(),
                ".comm %s, 4, 4\n",
                variable->getNameCString());

        } else if (entry->getKind() == SymbolEntry::KindEnum::kConstantKind) {
            int offset = m_symbol_manager.m_offset-4;
            m_symbol_manager.m_offset -= 4;
            entry->setStackOffset(offset);
            const Constant *constant = entry->getAttribute().constant();
            string result = constant->getConstantValueCString();
            if (result == "false") result = "0";
            else if (result == "true") result = "1";
            dumpInstructions(m_output_file.get(),
                "    li t0, %s\n"
                "    sw t0, %d(s0)\n",
                result.c_str(), offset
            );

        } else {
            cout << "fere" << endl;
            int offset = m_symbol_manager.m_offset-4;
            m_symbol_manager.m_offset -= 4;
            entry->setStackOffset(offset);
            dumpInstructions(m_output_file.get(),
                "    # Local variable '%s' at offset %d\n",
                variable->getNameCString(),
                offset
            );
            
        }
    }
}

void CodeGenerator::visit(VariableNode &p_variable) {
}

void CodeGenerator::visit(ConstantValueNode &p_constant_value) {
    string result = p_constant_value.getConstantValueCString();
    if (result == "false") result = "0";
    else if (result == "true") result = "1";
    dumpInstructions(m_output_file.get(),
                     "    li t0, %s\n"
                     "    li a0, %s\n",
                    result.c_str(), 
                    result.c_str()
    );
}

void CodeGenerator::visit(FunctionNode &p_function) {
    auto it = m_symbol_table_of_scoping_nodes.find(&p_function);
    if (it == m_symbol_table_of_scoping_nodes.end()) {
        std::cerr << "[Error] Missing symbol table for FunctionNode '"
                  << p_function.getNameCString() << "'\n";
        assert(false && "Missing symbol table for function");
    }
    m_symbol_manager.pushScope(std::move(it->second));

    dumpInstructions(m_output_file.get(),
        ".section    .text\n"
        "    .align 2\n"
        "    .globl %s\n"
        "    .type %s, @function\n"
        "%s:\n"
        "    # Function prologue (128 bytes)\n"
        "    addi sp, sp, -128\n"
        "    sw ra, 124(sp)\n"
        "    sw s0, 120(sp)\n"
        "    addi s0, sp, 128\n",
        p_function.getNameCString(),
        p_function.getNameCString(),
        p_function.getNameCString()
    );

    int arg_index = 1;
    for (auto &param_decl : p_function.getParameters()) {
        param_decl->accept(*this);
        for (int i=0 ; i < param_decl->getVariables().size(); i++){
            auto *param_entry = m_symbol_manager.lookup(param_decl->getVariables()[i]->getNameCString());
            int offset = param_entry->getStackOffset();
            if (arg_index < 8){
                dumpInstructions(m_output_file.get(),
                "    sw a%d, %d(s0)     # store param '%s' into offset %d\n",
                arg_index, offset,
                param_decl->getVariables()[i]->getNameCString(),
                offset
                );
            }else{
                dumpInstructions(m_output_file.get(),
                "    sw t%d, %d(s0)     # store param '%s' into offset %d\n",
                arg_index-6, offset,
                param_decl->getVariables()[i]->getNameCString(),
                offset
                );
            }
            arg_index++;
        }
    }

    std::string epilogue_label = p_function.getName() + "_epilogue";
    m_current_function_epilogue = epilogue_label;

    p_function.visitBodyChildNodes(*this);

    dumpInstructions(m_output_file.get(),
        "%s:\n"
        "    lw ra, 124(sp)\n"
        "    lw s0, 120(sp)\n"
        "    addi sp, sp, 128\n"
        "    jr ra\n"
        "    .size %s, .-%s\n",
        epilogue_label.c_str(),
        p_function.getNameCString(),
        p_function.getNameCString()
    );

    m_symbol_manager.popScope();
}




void CodeGenerator::visit(CompoundStatementNode &p_compound_statement) {
    auto it = m_symbol_table_of_scoping_nodes.find(&p_compound_statement);
    if (it != m_symbol_table_of_scoping_nodes.end()) {
       m_symbol_manager.pushScope(std::move(it->second));
    }
    p_compound_statement.visitChildNodes(*this);
    if (it != m_symbol_table_of_scoping_nodes.end()) {
        m_symbol_manager.popScope();
    }
}


void CodeGenerator::visit(PrintNode &p_print) {
    const auto &target_expr = p_print.getTarget();
    auto *target = dynamic_cast<const VariableReferenceNode *>(&target_expr);
    if (!target) { // assume is BinaryOperatorNode
        p_print.visitChildNodes(*this);
        dumpInstructions(m_output_file.get(),
                         "    jal ra, printInt\n"
        );
    }else{
        const SymbolEntry *entry = m_symbol_manager.lookup(target->getName());
        if (!entry) {
            assert(false && "Variable not found in symbol table");
            return;
        }

        if (entry->getLevel() == 0) {
            dumpInstructions(m_output_file.get(),
                "    la  t0, %s     # load address of global var\n"
                "    lw  t1, 0(t0)  # t1 = [global_var]\n"
                "    mv  a0, t1     # a0 = t1\n"
                "    jal ra, printInt\n",
                target->getNameCString());
        } else {
            int offset = entry->getStackOffset();
            dumpInstructions(m_output_file.get(),
                "    lw t0, %d(s0)\n"
                "    mv a0, t0\n"
                "    jal ra, printInt\n",
                offset);
        }
    }
}

void CodeGenerator::visit(BinaryOperatorNode &p_bin_op) {
    const ExpressionNode &left_operand = p_bin_op.getLeftOperand();
    
    if (auto *var_ref_node = dynamic_cast<const VariableReferenceNode *>(&left_operand)) {
         const std::string var_name = var_ref_node->getNameCString();
        const SymbolEntry *entry = m_symbol_manager.lookup(var_name);
        if (!entry) {
            std::cerr << "Error: Variable " << var_name << " not found in symbol table.\n";
            assert(false && "Variable not found in symbol table");
        }
        std::cout << "Loading left operand: " << var_name << " (Level " << entry->getLevel() << ")\n";
        if (entry->getLevel() == 0) {
            dumpInstructions(m_output_file.get(),
                             "    # Loading global variable %s\n"
                             "    la t0, %s\n"
                             "    lw t0, 0(t0)\n",
                             var_name.c_str(),
                             var_name.c_str());
        } else {
            int offset = entry->getStackOffset();
            dumpInstructions(m_output_file.get(),
                             "    # Loading local variable %s from offset %d\n"
                             "    addi t0, s0, %d\n"
                             "    lw t0, 0(t0)\n",
                             var_name.c_str(),
                             offset,
                             offset);
        }
    } else if (auto *const_val_node = dynamic_cast<const ConstantValueNode *>(&left_operand)) {
        std::cout << "Loading left operand: constant " << const_val_node->getConstantValueCString() << "\n";
        dumpInstructions(m_output_file.get(),
                         "    # Loading constant value %s\n"
                         "    li t0, %s\n",
                         const_val_node->getConstantValueCString(),
                         const_val_node->getConstantValueCString());
    } else if (auto *binary_node = dynamic_cast<const BinaryOperatorNode *>(&left_operand)) {
       const_cast<BinaryOperatorNode *>(binary_node)->accept(*this);
        dumpInstructions(m_output_file.get(),
                         "    mv t0, a0     # Move left operand result from a0 to t0\n");
    } else if (auto *fun_inv_node = dynamic_cast<const FunctionInvocationNode*>(&left_operand)){
        const_cast<FunctionInvocationNode*>(fun_inv_node)->accept(*this);
        dumpInstructions(m_output_file.get(),
                         "    mv t0, a0     # Move left operand result from a0 to t0\n");
    }

    dumpInstructions(m_output_file.get(),
                     "    # Pushing left operand result to stack\n"
                     "    addi sp, sp, -4\n"
                     "    sw t0, 0(sp)\n");
                     

    const ExpressionNode &right_operand = p_bin_op.getRightOperand();
    if (auto *var_ref_node = dynamic_cast<const VariableReferenceNode *>(&right_operand)) {
        const std::string var_name = var_ref_node->getNameCString();
        const SymbolEntry *entry = m_symbol_manager.lookup(var_name);
        if (!entry) {
            std::cerr << "Error: Variable " << var_name << " not found in symbol table.\n";
            assert(false && "Variable not found in symbol table");
        }
        std::cout << "Loading right operand: " << var_name << " (Level " << entry->getLevel() << ")\n";
        if (entry->getLevel() == 0) {
            dumpInstructions(m_output_file.get(),
                             "    # Loading global variable %s\n"
                             "    la t0, %s\n"
                             "    lw t0, 0(t0)\n",
                             var_name.c_str(),
                             var_name.c_str());
        } else {
            int offset = entry->getStackOffset();
            dumpInstructions(m_output_file.get(),
                             "    # Loading local variable %s from offset %d\n"
                             "    addi t0, s0, %d\n"
                             "    lw t0, 0(t0)\n",
                             var_name.c_str(),
                             offset,
                             offset);
        }
    } else if (auto *const_val_node = dynamic_cast<const ConstantValueNode *>(&right_operand)) {
        std::cout << "Loading right operand: constant " << const_val_node->getConstantValueCString() << "\n";
        dumpInstructions(m_output_file.get(),
                         "    # Loading constant value %s\n"
                         "    li t0, %s\n",
                         const_val_node->getConstantValueCString(),
                         const_val_node->getConstantValueCString());
    } else if (auto *binary_node = dynamic_cast<const BinaryOperatorNode *>(&right_operand)) {
        const_cast<BinaryOperatorNode *>(binary_node)->accept(*this);
        dumpInstructions(m_output_file.get(),
                         "    mv t0, a0     # Move right operand result from a0 to t0\n");
    }else if (auto *fun_inv_node = dynamic_cast<const FunctionInvocationNode*>(&right_operand)){
        const_cast<FunctionInvocationNode*>(fun_inv_node)->accept(*this);
        dumpInstructions(m_output_file.get(),
                         "    mv t0, a0     # Move left operand result from a0 to t0\n");
    }

    dumpInstructions(m_output_file.get(),
                     "    # Popping left operand result from stack to t1\n"
                     "    lw t1, 0(sp)\n"
                     "    addi sp, sp, 4\n");

    const std::string op = p_bin_op.getOpCString();
    if (op == "+") {
        dumpInstructions(m_output_file.get(),
                         "    # Performing addition: t0 = t1 + t0\n"
                         "    add t0, t1, t0\n"); // t0 = t1 + t0
    } else if (op == "-") {
        dumpInstructions(m_output_file.get(),
                         "    # Performing subtraction: t0 = t1 - t0\n"
                         "    sub t0, t1, t0\n"); // t0 = t1 - t0
    } else if (op == "*") {
        dumpInstructions(m_output_file.get(),
                         "    # Performing multiplication: t0 = t1 * t0\n"
                         "    mul t0, t1, t0\n"); // t0 = t1 * t0
    } else if (op == "/") {
        dumpInstructions(m_output_file.get(),
                         "    # Performing division: t0 = t1 / t0\n"
                         "    div t0, t1, t0\n"); // t0 = t1 / t0
    } else if (op == "=") {
        dumpInstructions(m_output_file.get(),
                         "    # Performing division: t0 == t1\n"
                         );
    } else if (op == "mod"){
        dumpInstructions(m_output_file.get(),
                         "    # Performing division: t0 = t1 mod t0\n"
                         "    rem t0, t1, t0\n");
    } else if (op == "and"){
        dumpInstructions(m_output_file.get(),
                         "    #and in here ccefcef\n"
                         "    mul t0, t1, t0\n"
                         );
    } else if (op == "or"){
        int i = m_symbol_manager.get_current_L();
        dumpInstructions(m_output_file.get(),
                         "    add t0, t1, t0\n"
                         "    li t1, 0\n"
                         "    beq t0,t1, L%d\n"
                         "    div t0, t0,t0\n"
                         "L%d:\n",
                         i, i

        );
    }

    dumpInstructions(m_output_file.get(),
        "    mv a0, t0     # Move binary operation result from t0 to a0\n");
}




void CodeGenerator::visit(UnaryOperatorNode &p_un_op) {
    p_un_op.visitChildNodes(*this);
    if (p_un_op.getOpCString()=="NOT"){
        dumpInstructions(m_output_file.get(),
                         "    li t1, 1\n"
                         "    sub t0, t0, t1\n"
                         "    mul t0, t0, t0\n"
                         "    mv a0 t0\n"
        );
    }else{
        dumpInstructions(m_output_file.get(),
                            "    # Performing subtraction: t0 = t1 - t0\n"
                            "    li t1, 0\n"
                            "    sub t0, t1, t0\n"
                            "    mv a0, t0\n");
    }
}

void CodeGenerator::visit(FunctionInvocationNode &p_func_invocation) {
    const std::string &func_name = p_func_invocation.getName();
    const auto &args = p_func_invocation.getArguments();

   
    int arg_reg = 1;
    for (auto &arg_expr : args) {
        arg_expr->accept(*this);
        if (arg_reg < 8) {
            dumpInstructions(m_output_file.get(),
                "    mv a%d, a0     # param #%d => a%d\n",
                arg_reg, arg_reg, arg_reg);
        }else{
            dumpInstructions(m_output_file.get(),
                "    mv t%d, a0     # param #%d => t%d\n",
                arg_reg-6, arg_reg, arg_reg-6);
        }
        arg_reg++;
    }

    dumpInstructions(m_output_file.get(),
        "    jal %s\n",
        func_name.c_str()
    );
   
}



void CodeGenerator::visit(VariableReferenceNode &p_variable_ref) {
    auto *entry = m_symbol_manager.lookup(p_variable_ref.getNameCString());
    int offset = entry->getStackOffset();
    if (entry->getLevel() > 0)
        dumpInstructions(m_output_file.get(),
                        "    #%s\n"
                        "    lw t0, %d(s0)\n"
                        "    mv a0, t0\n",
                        p_variable_ref.getNameCString(), offset
        );
    else{
        dumpInstructions(m_output_file.get(),
                         "    la  t0, %s\n"
                         "    lw  t0, 0(t0)\n"
                         "    mv  a0, t0\n",
                         p_variable_ref.getNameCString()
        );
    }
}

void CodeGenerator::visit(AssignmentNode &p_assignment) {
    const auto &lhs = p_assignment.getLvalue();
    const auto &rhs = p_assignment.getExpr();
    auto *c_rhs = dynamic_cast<const ConstantValueNode*>(&rhs);

    const SymbolEntry *entry = m_symbol_manager.lookup(lhs.getNameCString());
    if (!entry) {
        std::cerr << "Error: Variable " << lhs.getNameCString() << " not found in symbol table.\n";
        assert(false && "Variable not found in symbol table");
        return;
    }

    if (c_rhs) {
        if (entry->getLevel() == 0) {
            std::cout << "Assigning global variable " << lhs.getNameCString() << " to constant " 
                      << c_rhs->getConstantValueCString() << "\n";
            dumpInstructions(m_output_file.get(),
                "    # Assigning constant %s to global variable %s\n"
                "    la  t1, %s\n"       // load address of LHS into t1
                "    li  t0, %s\n"       // load immediate value into t0
                "    sw  t0, 0(t1)\n",   // store t0 into [t1]
                
                lhs.getNameCString(), c_rhs->getConstantValueCString(),
                lhs.getNameCString(), c_rhs->getConstantValueCString());
        } else {
            int offset = entry->getStackOffset();
            dumpInstructions(m_output_file.get(),
                "    li t0, %s\n"
                "    sw t0, %d(s0)\n",
                c_rhs->getConstantValueCString(),
                offset
                );
        }
    } else if (auto *b_rhs = dynamic_cast<const BinaryOperatorNode*>(&rhs)){
       if (b_rhs) {
            std::cout << "Assigning variable " << lhs.getNameCString() << " to expression.\n";
            this->visit(*const_cast<BinaryOperatorNode *>(b_rhs));

            if (entry->getLevel() == 0) {
                dumpInstructions(m_output_file.get(),
                                "    # Assigning expression result to global variable %s\n"
                                "    la  t1, %s\n"       // load address of LHS into t1
                                "    sw  t0, 0(t1)\n",   // store t0 into [t1]
                                lhs.getNameCString(),
                                lhs.getNameCString());
            } else {
                int offset = entry->getStackOffset();
                dumpInstructions(m_output_file.get(),
                                "    # Assigning expression result to local variable %s at offset %d\n"
                                "    addi t1, s0, %d\n"  // load address of LHS into t1
                                "    sw  t0, 0(t1)\n",   // store t0 into [t1]
                                lhs.getNameCString(),
                                offset,
                                offset);
            }
        } else {
            std::cerr << "Error: Unsupported RHS expression type for assignment to " 
                      << lhs.getNameCString() << "\n";
            assert(false && "Unsupported RHS expression type");
        }
    }else if (auto *func_invocation = dynamic_cast<const FunctionInvocationNode*>(&rhs)){
        const_cast<FunctionInvocationNode *>(func_invocation)->accept(*this);
        if (entry->getLevel() == 0) {
            dumpInstructions(m_output_file.get(),
                "    # Assigning function return value to global variable %s\n"
                "    la  t1, %s      # Load address of global variable %s into t1\n"
                "    sw  a0, 0(t1)\n",   // store a0 into [t1]
                lhs.getNameCString(),
                lhs.getNameCString(),
                lhs.getNameCString());
        } else {
            int offset = entry->getStackOffset();
            dumpInstructions(m_output_file.get(),
                "    # Assigning function return value to local variable %s at offset %d\n"
                "    addi t1, s0, %d # Load address of local variable %s into t1\n"
                "    sw  a0, 0(t1)\n",   // store a0 into [t1]
                lhs.getNameCString(),
                offset,
                offset,
                lhs.getNameCString());
        }
    }else if (auto *UON = dynamic_cast<const UnaryOperatorNode*>(&rhs)){
        const_cast<UnaryOperatorNode *>(UON)->accept(*this);
        if (entry->getLevel() == 0) {
            std::cout << "Assigning global variable " << lhs.getNameCString() << " to constant " 
                      << c_rhs->getConstantValueCString() << "\n";
            dumpInstructions(m_output_file.get(),
                "    # Assigning constant %s to global variable %s\n"
                "    la  t1, %s\n"       // load address of LHS into t1
                "    mv  t0, a0\n"       // load immediate value into t0
                "    sw  t0, 0(t1)\n",   // store t0 into [t1]
                
                lhs.getNameCString(), c_rhs->getConstantValueCString(),
                lhs.getNameCString());
        } else {
            int offset = entry->getStackOffset();
            dumpInstructions(m_output_file.get(),
                "    mv t0, a0\n"
                "    sw t0, %d(s0)\n",
                offset
                );
        }
    }
}



void CodeGenerator::visit(ReadNode &p_read) {
    auto name = p_read.getTarget().getNameCString();
    auto *entry = m_symbol_manager.lookup(name);
    if (entry->getLevel()>0)
        dumpInstructions(m_output_file.get(),
                         "    addi t0, s0, %d\n"
                         "    jal  ra, readInt\n"
                         "    sw   a0, 0(t0)\n",
                         entry->getStackOffset()
        );
    else
        dumpInstructions(m_output_file.get(),
                         "    la   t0, %s\n"
                         "    addi sp, sp, -4\n"
                         "    sw t0, 0(sp)\n"
                         "    jal  ra, readInt\n"
                         "    lw t0, 0(sp)\n"
                         "    addi sp, sp, 4\n"
                         "    sw a0, 0(t0)\n",
                         name
        );
}

void CodeGenerator::visit(IfNode &p_if) {
    auto *BON = dynamic_cast<BinaryOperatorNode*>(p_if.m_condition.get());
    int l = m_symbol_manager.get_current_L(); // 1
    int ll = m_symbol_manager.get_current_L(); // 2
    if (BON){
        cout << "8248" << endl;
        string op = BON->getOpCString();
        p_if.m_condition->accept(*this);
        string str;
        if (op=="<=") str = "bgt";
        else if (op=="=") str = "bne";
        else if (op==">") str = "ble";
        else if (op==">=") str = "blt";
        else if (op == "<") str = "bge";
        else if (op=="and" || op=="or"){
            dumpInstructions(m_output_file.get(),
                             "    mv t1, a0\n"
                             "    li t0, 0\n"
            );
            str = "ble";
        }
        dumpInstructions(m_output_file.get(),
                        "    %s t1, t0, L%d\n" // 2
                        "L%d:\n",              // 1
                        str.c_str(), ll, l
        );
    }else if (auto *VRN = dynamic_cast<VariableReferenceNode*>(p_if.m_condition.get())){
        auto *entry = m_symbol_manager.lookup(VRN->getNameCString());
        dumpInstructions(m_output_file.get(),
                         "    lw t0, %d(s0)\n"
                         "    li t1, 0\n"
                         "    ble t0, t1, L%d\n"
                         "L%d:\n",
                         entry->getStackOffset(), ll, l
        );

    }else{
        p_if.m_condition->accept(*this);
        dumpInstructions(m_output_file.get(),
                         "    li t1, 0\n"
                         "    mv t0, a0\n"
                         "    ble t0, t1, L%d\n"
                         "L%d:\n",
                         ll, l
        );

    }
    p_if.m_body->accept(*this);
    l = m_symbol_manager.get_current_L(); //3
    dumpInstructions(m_output_file.get(),
                     "    j L%d\n"
                     "L%d:\n",
                     l, ll
    );
    if (p_if.m_else_body) p_if.m_else_body->accept(*this);
    dumpInstructions(m_output_file.get(),
                     "L%d:\n",
                     l
    );
    
}

void CodeGenerator::visit(WhileNode &p_while) {
    dumpInstructions(m_output_file.get(),
                     "    #in while\n"
    );
    int i = m_symbol_manager.get_current_L(); // 3
    int j = m_symbol_manager.get_current_L(); // 4
    int k = m_symbol_manager.get_current_L(); // 5
    dumpInstructions(m_output_file.get(),
                     "L%d:\n",
                     i
    );
    p_while.m_condition->accept(*this);
    auto *BON = dynamic_cast<BinaryOperatorNode*>(p_while.m_condition.get());
    string op = BON->getOpCString();
    string str;
    if (op=="<=") str = "bgt";
    else if (op=="=") str = "bne";
    else if (op==">") str = "ble";
    else if (op==">=") str = "blt";
    else if (op == "<") str = "bge";
    else if (op=="and" || op=="or"){
        dumpInstructions(m_output_file.get(),
                            "    mv t1, a0\n"
                            "    li t0, 0\n"
        );
        str = "ble";
    }
    dumpInstructions(m_output_file.get(),
                     "    %s t1, t0, L%d\n"
                     "L%d:\n",
                     str.c_str(), k, j
    );
    p_while.m_body->accept(*this);
    dumpInstructions(m_output_file.get(),
                     "    j L%d\n"
                     "L%d:\n",
                     i, k
    );
}

void CodeGenerator::visit(ForNode &p_for) {
    dumpInstructions(m_output_file.get(),
                     "    #in for node\n"
                     
    );
    m_symbol_manager.pushScope(
        std::move(m_symbol_table_of_scoping_nodes.at(&p_for)));
    int i = m_symbol_manager.get_current_L(); // 6
    int j = m_symbol_manager.get_current_L(); // 7
    int k = m_symbol_manager.get_current_L(); // 8
    p_for.m_loop_var_decl->accept(*this);
    p_for.m_init_stmt->accept(*this);
    dumpInstructions(m_output_file.get(),
                     "L%d:\n",
                     i
    );
    auto name = p_for.m_loop_var_decl->getVariables()[0]->getNameCString();
    auto *entry = m_symbol_manager.lookup(name);
    int offset = entry->getStackOffset();
    p_for.m_end_condition->accept(*this);
    dumpInstructions(m_output_file.get(),
                     "    lw t1, %d(s0)\n"
                     "    bge t1, t0, L%d\n"
                     "L%d:\n",
                     offset, k, j
    );
    p_for.m_body->accept(*this);
    dumpInstructions(m_output_file.get(),
                     "    li t1, 1\n"
                     "    lw t0, %d(s0)\n"
                     "    add t0, t0, t1\n"
                     "    sw t0, %d(s0)\n"
                     "    j L%d\n"
                     "L%d:\n",
                     offset, offset, i, k
    );
    m_symbol_manager.popScope();
}

void CodeGenerator::visit(ReturnNode &p_return) {
    const ExpressionNode &return_expr = p_return.getReturnValue();

    p_return.visitChildNodes(*this);
    dumpInstructions(m_output_file.get(),
        "    # Jump to function epilogue\n"
        "    j %s\n",
        m_current_function_epilogue.c_str());
}